#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <vector>
#include <chrono>
#include <string>
#include <map>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class MapfExecuter : public rclcpp::Node
{
public:
    MapfExecuter() : Node("mapf_executer")
    {
        // Initialize action clients for both robots
        robot1_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "robotinobase1/navigate_to_pose");
        robot2_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "robotinobase2/navigate_to_pose");

        // Wait for action servers to be available
        while (!robot1_client_->wait_for_action_server(1s) || 
               !robot2_client_->wait_for_action_server(1s)) {
            if (!rclcpp::ok()) {
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for action servers to be available...");
        }

        // Initialize goal waypoints for both robots
        initializeWaypoints();

        // Start the navigation execution
        current_goal_index_ = 0;
        sendGoalsToRobots();
    }

private:
    // Action clients for both robots
    rclcpp_action::Client<NavigateToPose>::SharedPtr robot1_client_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr robot2_client_;

    // Waypoints for each robot
    std::vector<geometry_msgs::msg::PoseStamped> robot1_waypoints_;
    std::vector<geometry_msgs::msg::PoseStamped> robot2_waypoints_;

    // Current goal tracking
    size_t current_goal_index_;
    std::map<std::string, bool> goal_reached_;

    // Timer for timeout handling
    rclcpp::TimerBase::SharedPtr timeout_timer_;

    void initializeWaypoints()
    {
        // Define waypoints for Robot 1
        robot1_waypoints_ = {
            createPoseStamped(0.0, 0.0, 0.0),    // Start position
            createPoseStamped(2.0, 0.0, 0.0),    // Move forward
            createPoseStamped(2.0, 2.0, 1.57),   // Turn left and move
            createPoseStamped(0.0, 2.0, 3.14),   // Turn around
            createPoseStamped(0.0, 0.0, 0.0)     // Return to start
        };

        // Define waypoints for Robot 2 (different path to avoid collision)
        robot2_waypoints_ = {
            createPoseStamped(0.0, -1.0, 0.0),   // Start position
            createPoseStamped(-2.0, -1.0, 0.0),  // Move backward
            createPoseStamped(-2.0, 1.0, 1.57),  // Turn left and move
            createPoseStamped(0.0, 1.0, 3.14),   // Turn around
            createPoseStamped(0.0, -1.0, 0.0)    // Return to start
        };

        RCLCPP_INFO(this->get_logger(), "Initialized waypoints for both robots");
        RCLCPP_INFO(this->get_logger(), "Robot 1 has %zu waypoints", robot1_waypoints_.size());
        RCLCPP_INFO(this->get_logger(), "Robot 2 has %zu waypoints", robot2_waypoints_.size());
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

    void sendGoalsToRobots()
    {
        if (current_goal_index_ >= robot1_waypoints_.size() || 
            current_goal_index_ >= robot2_waypoints_.size()) {
            RCLCPP_INFO(this->get_logger(), "All waypoints completed!");
            return;
        }

        // Reset goal reached status
        goal_reached_["robot1"] = false;
        goal_reached_["robot2"] = false;

        // Send goal to Robot 1
        sendGoalToRobot("robot1", robot1_client_, robot1_waypoints_[current_goal_index_]);
        
        // Send goal to Robot 2
        sendGoalToRobot("robot2", robot2_client_, robot2_waypoints_[current_goal_index_]);

        RCLCPP_INFO(this->get_logger(), "Sent goal %zu to both robots", current_goal_index_ + 1);
    }

    void sendGoalToRobot(const std::string& robot_name, 
                        rclcpp_action::Client<NavigateToPose>::SharedPtr client,
                        const geometry_msgs::msg::PoseStamped& goal_pose)
    {
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal_pose;
        goal_msg.pose.header.stamp = this->get_clock()->now();

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        // Goal response callback
        send_goal_options.goal_response_callback =
            [this, robot_name](const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "%s goal was rejected by server", robot_name.c_str());
                } else {
                    RCLCPP_INFO(this->get_logger(), "%s goal accepted by server, waiting for result", robot_name.c_str());
                }
            };

        // Feedback callback
        send_goal_options.feedback_callback =
            [this, robot_name](GoalHandleNavigateToPose::SharedPtr,
                              const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
                auto distance_remaining = feedback->distance_remaining;
                RCLCPP_INFO(this->get_logger(), "%s distance remaining: %.2f", robot_name.c_str(), distance_remaining);
            };

        // Result callback
        send_goal_options.result_callback =
            [this, robot_name](const GoalHandleNavigateToPose::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "%s goal succeeded!", robot_name.c_str());
                        goal_reached_[robot_name] = true;
                        checkIfBothRobotsReached();
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "%s goal was aborted", robot_name.c_str());
                        goal_reached_[robot_name] = true;
                        checkIfBothRobotsReached();
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_ERROR(this->get_logger(), "%s goal was canceled", robot_name.c_str());
                        goal_reached_[robot_name] = true;
                        checkIfBothRobotsReached();
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "%s unknown result code", robot_name.c_str());
                        break;
                }
            };

        client->async_send_goal(goal_msg, send_goal_options);
    }

    void checkIfBothRobotsReached()
    {
        if (goal_reached_["robot1"] && goal_reached_["robot2"]) {
            RCLCPP_INFO(this->get_logger(), "Both robots reached their goals. Waiting 3 seconds before next goal...");
            
            // Start 3-second timeout timer
            timeout_timer_ = this->create_wall_timer(
                3s, 
                [this]() {
                    current_goal_index_++;
                    sendGoalsToRobots();
                    timeout_timer_->cancel();
                }
            );
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
