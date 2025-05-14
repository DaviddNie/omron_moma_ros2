#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "tm_msgs/srv/movement_request.hpp"  // Replace with your actual service name

class MoveitPathPlanningServer {
    public:
      MoveitPathPlanningServer(const rclcpp::Node::SharedPtr& node)
      : node_(node)
      {
        using namespace std::placeholders;
    
        RCLCPP_INFO(node_->get_logger(), "Starting MoveIt Path Planning Server...");
    
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "tmr_arm");
    
        service_ = node_->create_service<tm_msgs::srv::MovementRequest>(
          "/moveit_path_plan",
          std::bind(&MoveitPathPlanningServer::handle_request, this, _1, _2)
        );
      }
    
      void handle_request(
        const std::shared_ptr<tm_msgs::srv::MovementRequest::Request> request,
        std::shared_ptr<tm_msgs::srv::MovementRequest::Response> response)
      {
        RCLCPP_INFO(node_->get_logger(), "Received MoveIt path planning request.");
    
        const std::vector<double>& positions = request->positions;
    
        if (positions.size() != 6) {
            RCLCPP_ERROR(node_->get_logger(), "Expected 6 joint positions, got %zu", positions.size());
            response->success = false;
            return;
        }
    
        move_group_->setJointValueTarget(positions);
    
        // Optional constraints (example)
        moveit_msgs::msg::Constraints constraints;
        moveit_msgs::msg::JointConstraint joint_constraint;
        joint_constraint.joint_name = "shoulder_joint";
        joint_constraint.position = 0.0;
        joint_constraint.tolerance_above = 0.1;
        joint_constraint.tolerance_below = 0.1;
        joint_constraint.weight = 1.0;
        constraints.joint_constraints.push_back(joint_constraint);
        move_group_->setPathConstraints(constraints);
    
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
        if (success) {
          RCLCPP_INFO(node_->get_logger(), "Plan successful. Executing...");
          move_group_->execute(plan);
          response->success = true;
        } else {
          RCLCPP_WARN(node_->get_logger(), "Planning failed.");
          response->success = false;
        }
    
        move_group_->clearPathConstraints();
      }
    
    private:
      rclcpp::Node::SharedPtr node_;
      std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
      rclcpp::Service<tm_msgs::srv::MovementRequest>::SharedPtr service_;
    };
    
    int main(int argc, char** argv)
    {
      rclcpp::init(argc, argv);
      auto node = std::make_shared<rclcpp::Node>("moveit_path_planning_server");
      MoveitPathPlanningServer server(node);
      rclcpp::spin(node);
      rclcpp::shutdown();
      return 0;
    }
    