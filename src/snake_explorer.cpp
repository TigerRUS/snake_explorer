#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <queue>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::vector<geometry_msgs::PoseStamped> waypoints;
int current_goal_idx = 0;
float coverage_resolution = 1.0;
double goal_timeout = 20.0;
float padding = 1.0;
ros::Time goal_sent_time;
bool waiting_for_result = false;
bool has_map = false;
bool returned_home = false;

std::shared_ptr<MoveBaseClient> ac;

struct MapBounds {
    float min_x, max_x, min_y, max_y;
    
    bool contains(float x, float y) const {
        return x >= min_x && x <= max_x && y >= min_y && y <= max_y;
    }
    
    MapBounds intersection(const MapBounds& other) const {
        return {
            std::max(min_x, other.min_x),
            std::min(max_x, other.max_x),
            std::max(min_y, other.min_y),
            std::min(max_y, other.max_y)
        };
    }
};

MapBounds manual_bounds;

MapBounds findFreeSpaceBounds(const nav_msgs::OccupancyGrid& map) {
    MapBounds free_bounds = {FLT_MAX, -FLT_MAX, FLT_MAX, -FLT_MAX};
    bool first_free = false;
    
    for (int y = 0; y < map.info.height; ++y) {
        for (int x = 0; x < map.info.width; ++x) {
            int index = y * map.info.width + x;
            if (map.data[index] == 0) {
                float real_x = x * map.info.resolution + map.info.origin.position.x;
                float real_y = y * map.info.resolution + map.info.origin.position.y;
                
                if (!manual_bounds.contains(real_x, real_y)) continue;
                
                if (!first_free) {
                    free_bounds.min_x = free_bounds.max_x = real_x;
                    free_bounds.min_y = free_bounds.max_y = real_y;
                    first_free = true;
                } else {
                    free_bounds.min_x = std::min(free_bounds.min_x, real_x);
                    free_bounds.max_x = std::max(free_bounds.max_x, real_x);
                    free_bounds.min_y = std::min(free_bounds.min_y, real_y);
                    free_bounds.max_y = std::max(free_bounds.max_y, real_y);
                }
            }
        }
    }
    
    if (!first_free) {
        ROS_WARN("No free space found within manual bounds!");
        return manual_bounds;
    }
    
    MapBounds padded_bounds = {
        std::max(free_bounds.min_x + padding, manual_bounds.min_x),
        std::min(free_bounds.max_x - padding, manual_bounds.max_x),
        std::max(free_bounds.min_y + padding, manual_bounds.min_y),
        std::min(free_bounds.max_y - padding, manual_bounds.max_y)
    };
    
    return padded_bounds;
}

void sendGoal(int index) {
    if (index >= waypoints.size()) return;

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = waypoints[index];
    goal.target_pose.header.stamp = ros::Time::now();

    if (index + 1 < waypoints.size()) {
        double dx = waypoints[index + 1].pose.position.x - waypoints[index].pose.position.x;
        double dy = waypoints[index + 1].pose.position.y - waypoints[index].pose.position.y;
        double yaw = atan2(dy, dx);
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    } else {
        goal.target_pose.pose.orientation.w = 1.0;
    }

    ac->sendGoal(goal);
    goal_sent_time = ros::Time::now();
    waiting_for_result = true;
    ROS_INFO("Sent goal %d: (%.2f, %.2f)", index,
             goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
}

void advanceGoal() {
    if (returned_home) return;

    current_goal_idx++;
    if (current_goal_idx < waypoints.size()) {
        sendGoal(current_goal_idx);
    } else {
        ROS_INFO("All waypoints completed. Returning to home...");

        move_base_msgs::MoveBaseGoal home_goal;
        home_goal.target_pose.header.frame_id = "map";
        home_goal.target_pose.header.stamp = ros::Time::now();
        home_goal.target_pose.pose.position.x = 0.0;
        home_goal.target_pose.pose.position.y = 0.0;
        home_goal.target_pose.pose.orientation.w = 1.0;

        ac->sendGoal(home_goal);
        goal_sent_time = ros::Time::now();
        waiting_for_result = true;
    }
}

void checkGoalStatus() {
    if (!waiting_for_result || !has_map || returned_home) return;

    auto state = ac->getState();

    tf::StampedTransform transform;
    try {
        static tf::TransformListener listener;
        listener.lookupTransform("map", "base_link", ros::Time(0), transform);
        double robot_x = transform.getOrigin().x();
        double robot_y = transform.getOrigin().y();

        double goal_x;
        double goal_y;

        if (current_goal_idx < waypoints.size()) {
            goal_x = waypoints[current_goal_idx].pose.position.x;
            goal_y = waypoints[current_goal_idx].pose.position.y;
        } else {
            goal_x = 0.0;
            goal_y = 0.0;
        }

        double dist = hypot(goal_x - robot_x, goal_y - robot_y);

        if (dist < 0.2 && current_goal_idx + 1 < waypoints.size()) {
            ROS_INFO("Switching to next goal (distance: %.2f)", dist);
            ac->cancelGoal();
            advanceGoal();
            return;
        }
    } catch (tf::TransformException& ex) {
        ROS_WARN("TF error: %s", ex.what());
    }

    if (state.isDone()) {
        waiting_for_result = false;

        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal %d succeeded.", current_goal_idx);
        } else {
            ROS_WARN("Goal %d failed (%s). Skipping...", current_goal_idx, state.toString().c_str());
        }

        if (current_goal_idx >= waypoints.size()) {
            ROS_INFO("Returned home. Navigation complete.");
            returned_home = true;
			
			ros::shutdown();
            return;
        } else {
            advanceGoal();
        }
    } else {
        if (current_goal_idx < waypoints.size() && 
            (ros::Time::now() - goal_sent_time).toSec() > goal_timeout) {
            ROS_WARN("Goal %d timed out. Skipping...", current_goal_idx);
            ac->cancelGoal();
            advanceGoal();
        }
    }
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    const auto& map = *msg;
    waypoints.clear();
    has_map = true;

    MapBounds bounds = findFreeSpaceBounds(map);
    ROS_INFO("Working bounds (free space ∩ manual bounds): x[%.2f, %.2f], y[%.2f, %.2f]", 
             bounds.min_x, bounds.max_x, bounds.min_y, bounds.max_y);

    int width = map.info.width;
    int height = map.info.height;
    float resolution = map.info.resolution;
    float origin_x = map.info.origin.position.x;
    float origin_y = map.info.origin.position.y;

    for (float y = bounds.min_y; y <= bounds.max_y; y += coverage_resolution) {
        bool left_to_right = (int((y - bounds.min_y) / coverage_resolution) % 2 == 0);
        float x_start = left_to_right ? bounds.min_x : bounds.max_x;
        float x_end = left_to_right ? bounds.max_x : bounds.min_x;
        float x_step = left_to_right ? coverage_resolution : -coverage_resolution;

        for (float x = x_start; 
             (x_step > 0 && x <= x_end) || (x_step < 0 && x >= x_end); 
             x += x_step) {
            
            int map_x = (x - origin_x) / resolution;
            int map_y = (y - origin_y) / resolution;
            
            if (map_x < 0 || map_x >= width || map_y < 0 || map_y >= height)
                continue;
                
            int index = map_y * width + map_x;
            if (index < 0 || index >= map.data.size()) 
                continue;
            
            bool is_free = true;
            for (int dy = -1; dy <= 1 && is_free; ++dy) {
                for (int dx = -1; dx <= 1 && is_free; ++dx) {
                    int nx = map_x + dx;
                    int ny = map_y + dy;
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        int nidx = ny * width + nx;
                        if (map.data[nidx] != 0) {
                            is_free = false;
                        }
                    } else {
                        is_free = false;
                    }
                }
            }
            
            if (!is_free) continue;

            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.orientation.w = 1.0;
            waypoints.push_back(pose);
        }
    }

    ROS_INFO("Generated %lu waypoints", waypoints.size());

    if (!waypoints.empty()) {
        current_goal_idx = 0;
        sendGoal(current_goal_idx);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "snake_explorer");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("min_x", manual_bounds.min_x, -1000.0f);
    pnh.param("max_x", manual_bounds.max_x, 1000.0f);
    pnh.param("min_y", manual_bounds.min_y, -1000.0f);
    pnh.param("max_y", manual_bounds.max_y, 1000.0f);

    ROS_INFO("Manual bounds: x[%.2f, %.2f], y[%.2f, %.2f]",
             manual_bounds.min_x, manual_bounds.max_x,
             manual_bounds.min_y, manual_bounds.max_y);

    ac = std::make_shared<MoveBaseClient>("move_base", true);
    ROS_INFO("Waiting for move_base action server...");
    ac->waitForServer();
    ROS_INFO("Connected to move_base");

    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);
    ros::Rate rate(2.0);

    while (ros::ok()) {
        ros::spinOnce();
        checkGoalStatus();
        rate.sleep();
    }

    return 0;
}