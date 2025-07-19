#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;
using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

class MapfExecuter : public rclcpp::Node
{
public:
    MapfExecuter() : Node("mapf_executer")
    {
        // Initialize action clients for both robots  
        robot1_client_ = rclcpp_action::create_client<FollowWaypoints>(
            this, "robotinobase1/follow_waypoints");
        robot2_client_ = rclcpp_action::create_client<FollowWaypoints>(
            this, "robotinobase2/follow_waypoints");

        // Wait for action servers to be available
        while (!robot1_client_->wait_for_action_server(1s) || 
               !robot2_client_->wait_for_action_server(1s)) {
            if (!rclcpp::ok()) {
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for waypoint follower servers to be available...");
        }

        // Initialize waypoints and send to both robots
        initializeWaypoints();
        
        // Initialize timing variables
        robot1_finished_ = false;
        robot2_finished_ = false;
        
        sendWaypointsToRobots();
        
        // Start timing when we send the waypoints
        start_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Started navigation timer");
    }

private:
    // Action clients for both robots
    rclcpp_action::Client<FollowWaypoints>::SharedPtr robot1_client_;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr robot2_client_;

    // Waypoints for each robot
    std::vector<geometry_msgs::msg::PoseStamped> robot1_waypoints_;
    std::vector<geometry_msgs::msg::PoseStamped> robot2_waypoints_;

    // Timing variables
    std::chrono::steady_clock::time_point start_time_;
    bool robot1_finished_;
    bool robot2_finished_;

    void initializeWaypoints()
    {
        // Machine positions from world file (keeping 0.5m distance + 0.28m robot radius = 0.78m total)
        // Cap Station 1: (1.5, -1.5) facing -90° (west)
        // Cap Station 2: (1.5, 1.5) facing -90° (west) 
        // Ring Station 1: (-2.6, 0) facing 180° (south)
        // Ring Station 2: (2.6, 0) facing -180° (south)
        // Base Station: (-1.5, -1.5) facing -90° (west)
        // Delivery Station: (-1.36, 0.25) facing 0° (east)

        // Robot 1 uses Cap Station 1 and Ring Station 1
        robot1_waypoints_ = {
            // Cap station 1 Front (approach from east side)
            createPoseStamped(1.5 + 0.78, -1.5, 0.0),
            // Cap Station 1 back (approach from west side) 
            createPoseStamped(1.5 - 0.78, -1.5, 3.14159),
            // Ring station 1 front (approach from north side)
            createPoseStamped(-2.6, 0.0 + 0.78, -1.5708),
            // Base station (approach from east side)
            createPoseStamped(-1.5 + 0.78, -1.5, 0.0),
            // Ring station 1 front
            createPoseStamped(-2.6, 0.0 + 0.78, -1.5708),
            // Base station 
            createPoseStamped(-1.5 + 0.78, -1.5, 0.0),
            // Ring station 1 front
            createPoseStamped(-2.6, 0.0 + 0.78, -1.5708),
            // Ring station 1 back (approach from south side)
            createPoseStamped(-2.6, 0.0 - 0.78, 1.5708),
            // Ring station 1 front
            createPoseStamped(-2.6, 0.0 + 0.78, -1.5708),
            // Ring station 1 back
            createPoseStamped(-2.6, 0.0 - 0.78, 1.5708),
            // Cap station 1 front
            createPoseStamped(1.5 + 0.78, -1.5, 0.0),
            // Cap station 1 back
            createPoseStamped(1.5 - 0.78, -1.5, 3.14159),
            // Delivery station (approach from west side)
            createPoseStamped(-1.36 - 0.78, 0.25, 3.14159)
        };

        // Robot 2 uses Cap Station 2 and Ring Station 2
        robot2_waypoints_ = {
            // Cap station 2 Front (approach from east side)
            createPoseStamped(1.5 + 0.78, 1.5, 0.0),
            // Cap Station 2 back (approach from west side)
            createPoseStamped(1.5 - 0.78, 1.5, 3.14159),
            // Ring station 2 front (approach from north side)
            createPoseStamped(2.6, 0.0 + 0.78, -1.5708),
            // Base station (approach from east side)
            createPoseStamped(-1.5 + 0.78, -1.5, 0.0),
            // Ring station 2 front
            createPoseStamped(2.6, 0.0 + 0.78, -1.5708),
            // Base station
            createPoseStamped(-1.5 + 0.78, -1.5, 0.0),
            // Ring station 2 front
            createPoseStamped(2.6, 0.0 + 0.78, -1.5708),
            // Ring station 2 back (approach from south side)
            createPoseStamped(2.6, 0.0 - 0.78, 1.5708),
            // Ring station 2 front
            createPoseStamped(2.6, 0.0 + 0.78, -1.5708),
            // Ring station 2 back
            createPoseStamped(2.6, 0.0 - 0.78, 1.5708),
            // Cap station 2 front
            createPoseStamped(1.5 + 0.78, 1.5, 0.0),
            // Cap station 2 back
            createPoseStamped(1.5 - 0.78, 1.5, 3.14159),
            // Delivery station (approach from west side)
            createPoseStamped(-1.36 - 0.78, 0.25, 3.14159)
        };

        RCLCPP_INFO(this->get_logger(), "Initialized waypoints for both robots based on machine positions");
        RCLCPP_INFO(this->get_logger(), "Robot 1 has %zu waypoints (Cap1 + Ring1 + Base + Delivery)", robot1_waypoints_.size());
        RCLCPP_INFO(this->get_logger(), "Robot 2 has %zu waypoints (Cap2 + Ring2 + Base + Delivery)", robot2_waypoints_.size());
    }

    geometry_msgs::msg::PoseStamped createPoseStamped(double x, double y, double yaw)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->get_clock()->now();
        
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.orientation = tf2::toMsg(q);
        
        return pose;
    }

    void sendWaypointsToRobots()
    {
        // Send all waypoints to Robot 1
        sendWaypointsToRobot("Robot1", robot1_client_, robot1_waypoints_);
        
        // Send all waypoints to Robot 2
        sendWaypointsToRobot("Robot2", robot2_client_, robot2_waypoints_);
        
        RCLCPP_INFO(this->get_logger(), "Sent all waypoints to both robots");
    }

    void sendWaypointsToRobot(const std::string& robot_name,
                             rclcpp_action::Client<FollowWaypoints>::SharedPtr client,
                             const std::vector<geometry_msgs::msg::PoseStamped>& waypoints)
    {
        auto goal_msg = FollowWaypoints::Goal();
        goal_msg.poses = waypoints;

        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        
        // Goal response callback
        send_goal_options.goal_response_callback =
            [this, robot_name](const GoalHandleFollowWaypoints::SharedPtr & goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "%s waypoint following was rejected by server", robot_name.c_str());
                } else {
                    RCLCPP_INFO(this->get_logger(), "%s waypoint following accepted by server", robot_name.c_str());
                }
            };

        // Feedback callback
        send_goal_options.feedback_callback =
            [this, robot_name](GoalHandleFollowWaypoints::SharedPtr,
                              const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "%s is following waypoint %d", 
                           robot_name.c_str(), feedback->current_waypoint);
            };

        // Result callback
        send_goal_options.result_callback =
            [this, robot_name](const GoalHandleFollowWaypoints::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "%s completed all waypoints successfully!", robot_name.c_str());
                        markRobotFinished(robot_name);
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "%s waypoint following was aborted", robot_name.c_str());
                        markRobotFinished(robot_name);
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_ERROR(this->get_logger(), "%s waypoint following was canceled", robot_name.c_str());
                        markRobotFinished(robot_name);
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "%s unknown result code", robot_name.c_str());
                        break;
                }
            };

        client->async_send_goal(goal_msg, send_goal_options);
    }

    void markRobotFinished(const std::string& robot_name)
    {
        if (robot_name == "Robot1") {
            robot1_finished_ = true;
        } else if (robot_name == "Robot2") {
            robot2_finished_ = true;
        }

        // Check if both robots are finished
        if (robot1_finished_ && robot2_finished_) {
            auto end_time = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_);
            
            RCLCPP_INFO(this->get_logger(), "=== NAVIGATION COMPLETE ===");
            RCLCPP_INFO(this->get_logger(), "Total navigation time: %.3f seconds", duration.count() / 1000.0);
            RCLCPP_INFO(this->get_logger(), "Both robots finished their waypoints!");
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<MapfExecuter>();
    
    RCLCPP_INFO(node->get_logger(), "Starting MAPF Executer for 2 robots");
    RCLCPP_INFO(node->get_logger(), "This will send navigation goals to robotinobase1 and robotinobase2");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
