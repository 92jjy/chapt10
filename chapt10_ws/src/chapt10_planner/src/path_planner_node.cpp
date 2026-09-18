#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
#include <future>
#include <optional>
#include <chrono>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "chapt10_planner/path_planner_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace chapt10_planner
{   
    using namespace std::chrono_literals;

    PathPlanner::PathPlanner()
    :
    Node("planner_node"),
    qos_policy_map_(rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()),
    qos_policy_drawing_(rclcpp::QoS(rclcpp::KeepLast(100)).reliable()),
    qos_policy_(rclcpp::QoS(rclcpp::KeepLast(20)).reliable()),
    search_id_(0),
    start_pose_received_(false),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
    {   
        // Reserving memory for double buffering in path smoothing algorithm
        old_state_.reserve(1000);
        new_state_.reserve(1000);

        // Parameters declaration
        this->declare_parameter("robot_radius", 0.15);
        this->declare_parameter("frame_id", "map");
        this->declare_parameter("expansion_draw_buffer", 170);
        this->declare_parameter("marker_height", 0.01);
        this->declare_parameter("marker_color_rgb", std::vector<double>{1.0, 0.0, 0.0});
        this->declare_parameter("smoothing_factor", 0.1);
        this->declare_parameter("boundary_safety_margin", 0.1);

        this->get_parameter("robot_radius", robot_radius_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("expansion_draw_buffer", expansion_draw_buffer_size_);
        this->get_parameter("marker_height", marker_height_);
        this->get_parameter("marker_color_rgb", marker_color_rgb_);
        this->get_parameter("smoothing_factor", smoothing_factor_);
        this->get_parameter("boundary_safety_margin", boundary_safety_margin_);


        // Param on change callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &parameters)
            {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;

                bool need_inflation = false;

                for (const auto &param : parameters)
                {
                    if (param.get_name() == "frame_id"){
                        frame_id_ = param.as_string();
                    } else if (param.get_name() == "expansion_draw_buffer"){
                        expansion_draw_buffer_size_ = param.as_int();
                    } else if (param.get_name() == "marker_height"){
                        marker_height_ = param.as_double();
                    } else if (param.get_name() == "marker_color_rgb"){
                        auto value = param.as_double_array();
                        if (value.size() == 3){
                            for (const auto color : value){
                                if (color < 0.0 || color > 1.0){
                                    result.reason = "Each color is set by value: 0.0 - 1.0";
                                    result.successful = false;
                                    return result;
                                } 
                            }
                            marker_color_rgb_ = value;
                        } else {
                            result.reason = "Color vector has to be 3-elemet vector!";
                            result.successful = false;
                            return result;
                        }
                    } else if (param.get_name() == "robot_radius"){
                        robot_radius_ = param.as_double();
                        need_inflation = true;
                    } else if (param.get_name() == "smoothing_factor"){
                        auto val = param.as_double();
                        if (val < 0.001 || val > 1.0){
                            result.reason = "Smothing_factor must be between 0.001 and 1.0!";
                            result.successful = false;
                            return result;
                        } 
                        smoothing_factor_ = val;
                    } else if (param.get_name() == "boundry_safety_margin"){
                        boundary_safety_margin_ = param.as_double();
                        need_inflation = true;
                    }
                }
                if (map_ && need_inflation){
                    inflated_map_ = inflate_map(map_);
                    RCLCPP_INFO(this->get_logger(), "Map re-inflated due to parameter change!");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Parameter updated. Inflation skipped.");
                }
                return result;
            }
        );

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            qos_policy_map_,
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg)
            {                
                map_ = msg;

                inflated_map_ = inflate_map(map_);

                RCLCPP_INFO(this->get_logger(), "Map received!\n Dimensions[m]: %.2fm x %.2fm \n Dimensions[pxl]: %d x %d",
                    map_->info.width * map_->info.resolution,
                    map_->info.height * map_->info.resolution,
                    map_->info.width,
                    map_->info.height
                );
            }
);

        start_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose",
            qos_policy_,
            [this](geometry_msgs::msg::PoseWithCovarianceStamped msg)
            {
                start_pose_ = msg;
                start_pose_received_ = true;
                RCLCPP_INFO(this->get_logger(), "Start pose received! \n Coordinates: X = %f, Y = %f",
                    start_pose_.pose.pose.position.x, start_pose_.pose.pose.position.y);

                // Clearing map visualization with arrival od new start pose.
                clear_map();
                erase_path_and_draw_start();
            }
        );

        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",
            qos_policy_,
            [this](geometry_msgs::msg::PoseStamped msg)
            {
                target_pose_ = msg;

                RCLCPP_INFO(this->get_logger(), "Goal pose received! \n Coordinates: X = %f, Y = %f", 
                    target_pose_.pose.position.x, target_pose_.pose.position.y);
                if (!start_pose_received_)
                {
                    try
                    {
                        const auto tf = tf_buffer_.lookupTransform(frame_id_, "base_footprint", tf2::TimePointZero);
                        start_pose_.header = tf.header;
                        start_pose_.pose.pose.position.x = tf.transform.translation.x;
                        start_pose_.pose.pose.position.y = tf.transform.translation.y;
                        start_pose_.pose.pose.orientation = tf.transform.rotation;
                        start_pose_received_ = true;
                        RCLCPP_INFO(this->get_logger(),
                            "No /initialpose, using TF pose as start: X = %f, Y = %f",
                            start_pose_.pose.pose.position.x, start_pose_.pose.pose.position.y);
                    }
                    catch (const tf2::TransformException & ex)
                    {
                        RCLCPP_INFO(this->get_logger(), "Start_pose_ is missing (%s)", ex.what());
                    }
                }

                // We start calculation only if we are certain that we have both poses, and calculation starts with arrival of target pose.    
                if (start_pose_received_){
                    if (planning_future_.valid()){
                        auto status = planning_future_.wait_for(0s);

                        if (status != std::future_status::ready){
                            RCLCPP_WARN(this->get_logger(), "Busy calculating path, wait to complete!");
                            return;
                        } 
                    } 
                    // Launching async path planning
                    planning_future_ = std::async(
                        std::launch::async, 
                        [this,map = inflated_map_]()
                        {
                            auto result = find_path(map);
                            return result;
                        }
                    );
                    
                } else {
                    RCLCPP_INFO(this->get_logger(), "Start_pose_ is missing, make sure to indicate start_pose_ before target_pose_.");
                }    
            }
        );

        drawing_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/draw",
            qos_policy_map_
        );

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/path",
            qos_policy_map_
        );

        check_if_path_solved_ = this->create_wall_timer(
            50ms, 
            [this]()->void{
                std::vector<std::vector<int>> copy_of_parcels_to_draw;

                {
                    // Swapping shared buffer with local copy for drawing
                    std::lock_guard<std::mutex> lock(drawing_mutex_);
                    if(!shared_expansion_draw_buffer_.empty())
                    {
                        copy_of_parcels_to_draw.swap(shared_expansion_draw_buffer_);
                    }
                }

                for (auto draw_package : copy_of_parcels_to_draw)
                {
                    draw_expansion(draw_package);
                }

                if (planning_future_.valid()){
                    auto status = planning_future_.wait_for(0s);

                    // If path found, publish it
                    if (status == std::future_status::ready)
                    {
                        auto result = planning_future_.get();
                        
                        if (result.has_value()){
                            path_ = std::move(result.value());
                            path_publish(path_);
                            RCLCPP_INFO(this->get_logger(), "Path found with %zu points.", path_.size());
                        } else {
                            path_.clear();
                            RCLCPP_WARN(this->get_logger(), "Path not found.");
                        }
                    } 
                }
            }
        );
    }

    std::optional<std::vector<geometry_msgs::msg::PoseStamped>> PathPlanner::find_path(std::shared_ptr<nav_msgs::msg::OccupancyGrid> inflated_map)
    {
        if(!inflated_map) {
            RCLCPP_INFO(this->get_logger(), "Map is missing, waiting for /map...");
            return std::nullopt;
        } 
        int local_width = inflated_map->info.width;
        int local_height = inflated_map->info.height;
        float local_res = inflated_map->info.resolution;
        double local_origin_x = inflated_map->info.origin.position.x;
        double local_origin_y = inflated_map->info.origin.position.y;
        int local_expansion_draw_buffer_size = expansion_draw_buffer_size_;

        if (map_nodes_.size() != static_cast<size_t>(local_width * local_height)){
            map_nodes_.resize(local_width * local_height);
        }
        // Incrementing id of each path.
        search_id_ += 1;
        // Clearing map in main loop to erase all markers in case only target pose has been changed
        clear_map();
        erase_path_and_draw_start();
        // Buffer for collecting indexes for markers to be drawn
        std::vector<int> expansion_draw_buffer;
        // Priority_queue of current neigbours
        std::priority_queue<GridNode, std::vector<GridNode>, CompareNodes> open_list;
        // Found path
        std::vector<geometry_msgs::msg::PoseStamped> path;
        // Local copy of poses for calculations
        GridNode start_pose;
        GridNode target_pose;

        // Start initialization
        start_pose.index = world_to_index(
            start_pose_.pose.pose.position.x,
            start_pose_.pose.pose.position.y,
            local_origin_x,
            local_origin_y,
            local_res, local_width
        );
        target_pose.index = world_to_index(
            target_pose_.pose.position.x,
            target_pose_.pose.position.y,
            local_origin_x,
            local_origin_y,
            local_res,
            local_width
        );
        
        if (index_belong_to_map(start_pose.index, local_width, local_height) && index_belong_to_map(target_pose.index, local_width, local_height) && not_obstacle(target_pose.index, inflated_map)){
            // Continuation of initialization of start pose
            start_pose.h_cost = calculate_distance(start_pose.index, target_pose.index, local_width);
            start_pose.g_cost = 0.0;
            start_pose.f_cost = start_pose.g_cost + start_pose.h_cost;

            map_nodes_[start_pose.index] = start_pose;
            expansion_draw_buffer.push_back(start_pose.index);
            open_list.push(start_pose);

            // Core loop
            while (!open_list.empty())
            {
                // Current best move in queue
                auto current_top = open_list.top();
                open_list.pop();

                if (map_nodes_[current_top.index].already_visited == search_id_){
                    continue;
                } else if (current_top.index == target_pose.index){
                    map_nodes_[current_top.index].already_visited = search_id_;
                    map_nodes_[current_top.index].parent_index = current_top.parent_index;

                    if(expansion_draw_buffer.size() < static_cast<size_t>(local_expansion_draw_buffer_size))
                    {
                        expansion_draw_buffer.push_back(current_top.index);
                    } else {
                        expansion_draw_buffer.push_back(current_top.index);
                        std::lock_guard<std::mutex> lock(drawing_mutex_);
                        {
                            shared_expansion_draw_buffer_.push_back(expansion_draw_buffer);
                        }
                        expansion_draw_buffer.clear();
                    }
                    break;
                } else{
                    map_nodes_[current_top.index].already_visited = search_id_;
                    map_nodes_[current_top.index].parent_index = current_top.parent_index;

                    if(expansion_draw_buffer.size() < static_cast<size_t>(local_expansion_draw_buffer_size))
                    {
                        expansion_draw_buffer.push_back(current_top.index);
                    } else {
                        expansion_draw_buffer.push_back(current_top.index);
                        std::lock_guard<std::mutex> lock(drawing_mutex_);
                        {
                            shared_expansion_draw_buffer_.push_back(expansion_draw_buffer);
                        }
                        expansion_draw_buffer.clear();
                    }
                }

                // Mapping all of neibours of current position
                auto current_top_xy = index_to_grid(current_top.index, local_width);

                for (int dx = current_top_xy.first - 1; dx < current_top_xy.first + 2; ++dx)
                {
                    for (int dy = current_top_xy.second - 1; dy < current_top_xy.second + 2; ++dy)
                    {   
                        // Validation of neibours coords (if they are parto of map and if they aren't obstacle)    
                        if (coordinates_validation(dx, dy, local_width, local_height)){
                            int index = dx + dy * local_width;
                            if (index == current_top.index){continue;}
                            else if (not_obstacle(index, inflated_map)){
                                GridNode new_neibour;
                                new_neibour.index = index;
                                new_neibour.h_cost = calculate_distance(new_neibour.index, target_pose.index, local_width);
                                new_neibour.g_cost = calculate_distance(current_top.index, new_neibour.index, local_width) + current_top.g_cost;
                                new_neibour.f_cost = new_neibour.h_cost + new_neibour.g_cost;
                                new_neibour.parent_index = current_top.index;

                                open_list.push(new_neibour);
                            }
                        }
                    }
                }
            }

            // Reconstructing path going backwards by parent_index of each GridNode.
            if (!open_list.empty()) 
            { 
                auto going_back_index = target_pose.index; 
                while (going_back_index != start_pose.index)
                {
                    going_back_index = map_nodes_[going_back_index].parent_index;
                    auto back_xy = index_to_grid(going_back_index, local_width);
                    double x_word = back_xy.first * local_res + local_origin_x;
                    double y_word = back_xy.second * local_res + local_origin_y;

                    geometry_msgs::msg::PoseStamped road_pice;

                    road_pice.pose.position.x = x_word;
                    road_pice.pose.position.y = y_word;
                    road_pice.pose.position.z = marker_height_ + 0.01;
                    road_pice.pose.orientation.w = 1.0;
                    road_pice.header.stamp = this->now();
                    road_pice.header.frame_id = frame_id_;

                    path.push_back(road_pice);
                }

                std::reverse(path.begin(), path.end());
                path = smooth_path(path, inflated_map);
                return path;
            } else {
                RCLCPP_WARN(this->get_logger(), "There is no available path!");
                return std::nullopt;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "There is no available path!");
            return std::nullopt;
        }
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr PathPlanner::inflate_map(const nav_msgs::msg::OccupancyGrid::SharedPtr &map)
    {
        if (!map){
            RCLCPP_WARN(this->get_logger(), "Empty map object!");
            return nullptr;
        }
        int local_width = map->info.width;
        int local_height = map->info.height;
        float local_res = map->info.resolution;
        double local_robot_radius = robot_radius_;
        // Creating copy inside for safety
        auto inflated_map = std::make_shared<nav_msgs::msg::OccupancyGrid>(*map);
        // Fifo queue for expansion wave calculation (works similar to conveyor belt)
        std::queue<int> edge_expansion_indexes;
        // Grid for indicating if we already visited this pxl or not (-1 if we haven't parent_index if we have)
        std::vector<int> parent_obstacle_index(local_width * local_height, -1);
        // Robot radius in pixels for calculations, we use ceil to raund up to integral value.
        auto robot_radius_pxl = std::ceil(local_robot_radius / local_res) + boundary_safety_margin_ / local_res;

        // Edge detection loop. We only add to queue those pixels (it's indexes) that are obstacles and at the same time borders to free pixels.
        for (int index = 0; index < local_width * local_height; ++index)
        {
            if (map->data[index] != 0){
                auto grid_index = index_to_grid(index, local_width);

                auto right_value = -1;
                auto left_value = -1;
                auto top_value = -1;
                auto bottom_value = -1;
                // Neibours mapping with Manhattan pattern.
                if (coordinates_validation(grid_index.first + 1, grid_index.second, local_width, local_height)){right_value = map->data[index + 1];}
                if (coordinates_validation(grid_index.first - 1, grid_index.second, local_width, local_height)){left_value = map->data[index - 1];}
                if (coordinates_validation(grid_index.first, grid_index.second + 1, local_width, local_height)){top_value = map->data[index + local_width];}
                if (coordinates_validation(grid_index.first, grid_index.second - 1, local_width, local_height)){bottom_value = map->data[index - local_width];}
                // Checks if one of neibours is free pixel. If so we add this pixel to queue (not the neibour but it's parent)
                if (right_value == 0 || left_value == 0 || top_value == 0 || bottom_value == 0)
                {
                    edge_expansion_indexes.push(index);
                    parent_obstacle_index[index] = index;
                }
            }
        }
        // Once we have prepared queue with all edges on the map we can poreceed expansion with BFS - Multi-Source Bredth-First Search algorithm.
        while(!edge_expansion_indexes.empty())
        {
            // Current index from queue.
            auto current = edge_expansion_indexes.front();
            // Current index in grid metrics (xy).
            auto current_coordinates = index_to_grid(current, local_width);
            edge_expansion_indexes.pop();
            // Parent index of current index (mentioned above to controll if we already visited this position).
            auto parent_index = parent_obstacle_index[current];
            // Parent index of current index in grid metrics.
            auto parent_coordinates = index_to_grid(parent_index, local_width);

            // For each current index we check it's neibours with Manhattan patern. If it belongs to map and we haven't visited it yet (!= -1) and 
            // it's distance to parent is <= than (robot radius + safety margin) in pixels we expand our map by this neibour, add it to queue to then check 2 raw of neibours that bordes with 1 raw of neibours that we've checked and so on.
            const int dx[4] = {1, -1, 0, 0};
            const int dy[4] = {0, 0, 1, -1};

            for(int i = 0; i <= 3; ++i){
                if (coordinates_validation(current_coordinates.first + dx[i], current_coordinates.second + dy[i], local_width, local_height)){
                    if (parent_obstacle_index[current + dx[i] + dy[i] * local_width] == -1 &&
                        std::hypot((current_coordinates.first + dx[i] - parent_coordinates.first), (current_coordinates.second + dy[i] - parent_coordinates.second)) <= robot_radius_pxl)
                    {
                        parent_obstacle_index[current + dx[i] + dy[i] * local_width] = parent_index;
                        edge_expansion_indexes.push(current + dx[i] + dy[i] * local_width);
                        inflated_map->data[current + dx[i] + dy[i] * local_width] = 100;
                    }
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Map inflation proceeded!");
        return inflated_map;
    }

    std::vector<geometry_msgs::msg::PoseStamped> PathPlanner::smooth_path(std::vector<geometry_msgs::msg::PoseStamped> &path, nav_msgs::msg::OccupancyGrid::SharedPtr &inflated_map)
    {
        int local_width = inflated_map->info.width;
        float local_res = inflated_map->info.resolution;
        double local_origin_x = inflated_map->info.origin.position.x;
        double local_origin_y = inflated_map->info.origin.position.y;

        old_state_.resize(path.size());
        new_state_.resize(path.size());
        // We do not smooth start and target poses, so we just copy them to new vector and then we do smoothing of inner points.
        for (size_t index = 0; index < path.size(); ++index)
        {   
            if (index == 0){
                new_state_[index].first = path[index].pose.position.x;
                new_state_[index].second = path[index].pose.position.y;

            } else if (index == old_state_.size() - 1){
                new_state_[index].first = path[index].pose.position.x;
                new_state_[index].second = path[index].pose.position.y;
            } 
            old_state_[index].first = path[index].pose.position.x;
            old_state_[index].second = path[index].pose.position.y;
        }
        for (int i = 0; i < (smoothing_factor_ * 1000); ++i)
        {
            for (size_t index = 0; index < old_state_.size(); ++index)
            {
                if (index == 0 || index == old_state_.size() - 1) {continue;}
                
                // Calculating arithmetic averages values of neibours of current index
                auto avrg_x = (old_state_[index + 1].first + old_state_[index - 1].first) / 2.0;
                auto avrg_y = (old_state_[index + 1].second + old_state_[index - 1].second) / 2.0;
                auto alfa = 1.0;
                // Streched average placeholder for sitiation when avr value can not be accepted due to the fact that it produces colision with obstacle
                auto stretched_old_x = avrg_x; 
                auto stretched_old_y = avrg_y;

                if (not_obstacle(world_to_index(avrg_x, avrg_y, local_origin_x, local_origin_y, local_res, local_width), inflated_map))
                {
                    new_state_[index].first = avrg_x;
                    new_state_[index].second = avrg_y;
                } else{
                    // When avergae value creates colision with object we strech it out from object 
                    while(!not_obstacle(world_to_index(stretched_old_x, stretched_old_y, local_origin_x, local_origin_y, local_res, local_width), inflated_map))
                    {
                        stretched_old_x = old_state_[index].first + (avrg_x - old_state_[index].first) * alfa;
                        stretched_old_y = old_state_[index].second + (avrg_y - old_state_[index].second) * alfa;
                        
                        if (alfa > 0.0){
                            alfa -= 0.05;
                        } else {
                            break;
                        }
                    }
                    new_state_[index].first = stretched_old_x;
                    new_state_[index].second = stretched_old_y;
                }
            } 
            // Remaping all new positions into old vector (update of reference vector), it's necesary to have 2 work vectors to avoid oscilations
            // in situations as: when we work on 2nd element so we do calculation using 1st and 3rd elements we actualize the 2nd element and 
            // start calculation on 3rd, but when using only one vector we proceed 3rd element calculations using already modified 2nd element which,
            // may lead to wierd behaviour. 
            old_state_.swap(new_state_);
        }
        for (size_t index = 0; index < old_state_.size(); ++index)
        {
            if (index == 0 || index == old_state_.size() - 1) {continue;}

            path[index].pose.position.x = old_state_[index].first;
            path[index].pose.position.y = old_state_[index].second;
        }
        return path;
    }

    double PathPlanner::calculate_distance(int start_index, int end_index, int width) const
    {
        auto start_xy = index_to_grid(start_index, width);
        auto end_xy = index_to_grid(end_index, width);

        double dx = static_cast<double>(end_xy.first - start_xy.first);
        double dy = static_cast<double>(end_xy.second - start_xy.second);

        return std::hypot(dx,dy);
    }

    bool PathPlanner::index_belong_to_map(int index, int width, int height) const
    {
        if (index < 0 || index >= (width* height))
        {
            return false;
        } else {
            return true;
        }
    }

    bool PathPlanner::not_obstacle(int index, const nav_msgs::msg::OccupancyGrid::SharedPtr &map) const
    {
        if (map->data[index] != 0)
        {
            return false;
        } else {
            return true;
        }
    }

    bool PathPlanner::coordinates_validation(int x, int y, int width, int height) const
    {
        if(x < 0 || x >= width || y < 0 || y >= height)
        {
            return false;
        } else {
            return true;
        }
    }

    int PathPlanner::world_to_index(double x, double y, double origin_x, double origin_y, float resolution, int width) const
    {
        double dx = x - origin_x;
        int index_x = dx / resolution;
        double dy = y - origin_y;
        int index_y = dy / resolution;

        int index = index_x  + index_y * width;

        return index;
    }

    std::pair<int, int> PathPlanner::index_to_grid(int index, int width) const
    {
        int x = index % width;
        int y = index / width;

        return {x, y};
    }

    void PathPlanner::draw_expansion(std::vector<int> &draw_expansion_buffer) const
    {
        if (!map_) return;

        static int counter = 0;

        visualization_msgs::msg::Marker marker;

        marker.id = counter;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = this->now();
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Setting marker look
        marker.scale.x = map_->info.resolution;
        marker.scale.y = map_->info.resolution;
        marker.scale.z = marker_height_;
        marker.color.r = marker_color_rgb_[0];
        marker.color.g = marker_color_rgb_[1];
        marker.color.b = marker_color_rgb_[2];
        marker.color.a = 1.0; //Solid color (without transparency)

        for (int index : draw_expansion_buffer)
        {
            geometry_msgs::msg::Point p;

            auto xy = index_to_grid(index, map_->info.width);
            double half_res = map_->info.resolution / 2;
            p.x = xy.first * map_->info.resolution + map_->info.origin.position.x + half_res;
            p.y = xy.second * map_->info.resolution + map_->info.origin.position.y + half_res;
            p.z = marker.scale.z / 2;

            marker.points.push_back(p);
        }
        draw_expansion_buffer.clear();
        drawing_pub_->publish(marker);
        counter++;
        
    }

    void PathPlanner::clear_map() const
    {
        visualization_msgs::msg::Marker clear_msg;
        clear_msg.header.frame_id = frame_id_;
        clear_msg.header.stamp = this->now();
        clear_msg.action = visualization_msgs::msg::Marker::DELETEALL; 
        drawing_pub_->publish(clear_msg);
    }

    void PathPlanner::erase_path_and_draw_start() const
    {
        nav_msgs::msg::Path empty_path;
        geometry_msgs::msg::PoseStamped road_pice;

        road_pice.pose.position.x = start_pose_.pose.pose.position.x;
        road_pice.pose.position.y = start_pose_.pose.pose.position.y;
        road_pice.pose.position.z = marker_height_ + 0.01;
        road_pice.pose.orientation.w = 1.0;
        road_pice.header.stamp = this->now();
        road_pice.header.frame_id = frame_id_;

        empty_path.poses.push_back(road_pice);

        geometry_msgs::msg::PoseStamped road_pice_2 = road_pice;
        geometry_msgs::msg::PoseStamped road_pice_3 = road_pice;
        road_pice_2.pose.position.x += robot_radius_; 
        road_pice_3.pose.position.x -= robot_radius_; 

        empty_path.poses.push_back(road_pice_2);
        empty_path.poses.push_back(road_pice_3);

        empty_path.header.frame_id = frame_id_; 
        empty_path.header.stamp = this->now();
        path_pub_->publish(empty_path);
    }

    void PathPlanner::path_publish(const std::vector<geometry_msgs::msg::PoseStamped>& path) const
    {
        nav_msgs::msg::Path final_path;

        final_path.header.frame_id = frame_id_;
        final_path.header.stamp = this->now();

        final_path.poses = path;

        path_pub_->publish(final_path);
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<chapt10_planner::PathPlanner>());
    rclcpp::shutdown();
}
