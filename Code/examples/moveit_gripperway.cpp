#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <gripper_srv/srv/gripper_service.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("joint_goal", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  // Create a ROS logger
  auto const logger = rclcpp::get_logger("joint_goal");
  auto gripper_client = node->create_client<gripper_srv::srv::GripperService>("gripper_service");
  // We spin up a SingleThreadedExecutor so we can get current joint values later
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt Move Group Interface for panda arm
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_with_gripper");
  std::vector<double> joint_group_positions2 = move_group_interface.getCurrentJointValues();

  // Sets the first joint value
  joint_group_positions2[0] = 0.0;  // radians 1.53858709,-1.6697823,   2.13969785, -2.20202078, -1.58471042, -0.0138014
  joint_group_positions2[1] = -1.57;
  joint_group_positions2[2] = 0.0;
  joint_group_positions2[3] = -1.57;
  joint_group_positions2[4] = 0.0;
  joint_group_positions2[5] = 0.0;
  move_group_interface.setJointValueTarget(joint_group_positions2);

  // Create a plan to these joint values and check if that plan is successful
  moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
  bool success3 = (move_group_interface.plan(my_plan3) == moveit::core::MoveItErrorCode::SUCCESS);

  // If the plan is successful, execute the plan
  if(success3) {
    move_group_interface.execute(my_plan3);
    while (!gripper_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(logger, "Gripper service not available, waiting again...");
    }
    // Create a request to close the gripper
    auto request = std::make_shared<gripper_srv::srv::GripperService::Request>();
    request->position = 0;  
    request->speed = 255;
    request->force = 255;
    // Call the gripper service asynchronously
    auto future = gripper_client->async_send_request(request);

    // Wait for the response
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Get all joint positions
  std::vector<double> joint_group_positions = move_group_interface.getCurrentJointValues();

  // Sets the first joint value
  joint_group_positions[0] = 1.4964273;  // radians 1.53858709,-1.6697823,   2.13969785, -2.20202078, -1.58471042, -0.0138014
  joint_group_positions[1] = -1.55256919;
  joint_group_positions[2] = 2.39133245;
  joint_group_positions[3] = -2.43049063;
  joint_group_positions[4] = -1.58589822;
  joint_group_positions[5] = -0.0773905;
  move_group_interface.setJointValueTarget(joint_group_positions);

  // Create a plan to these joint values and check if that plan is successful
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // If the plan is successful, execute the plan
  if(success) {
    move_group_interface.execute(my_plan);
    while (!gripper_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(logger, "Gripper service not available, waiting again...");
    }
    // Create a request to close the gripper
    auto request = std::make_shared<gripper_srv::srv::GripperService::Request>();
    request->position = 255;  
    request->speed = 255;
    request->force = 255;
    // Call the gripper service asynchronously
    auto future = gripper_client->async_send_request(request);

    // Wait for the response
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  joint_group_positions[0] = 1.93897378;  // radians 1.53858709,-1.6697823,   2.13969785, -2.20202078, -1.58471042, -0.0138014
  joint_group_positions[1] = -1.39441512;
  joint_group_positions[2] = 2.20265466;
  joint_group_positions[3] = -2.40332188;
  joint_group_positions[4] = -1.59453327;
  joint_group_positions[5] = -0.36565638;
  move_group_interface.setJointValueTarget(joint_group_positions);
  // Create a plan to these joint values and check if that plan is successful
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
  bool success2 = (move_group_interface.plan(my_plan2) == moveit::core::MoveItErrorCode::SUCCESS);

  // If the plan is successful, execute the plan
  if(success2) {
    move_group_interface.execute(my_plan2);
        while (!gripper_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(logger, "Gripper service not available, waiting again...");
    }
    // Create a request to close the gripper
    auto request = std::make_shared<gripper_srv::srv::GripperService::Request>();
    request->position = 0;  
    request->speed = 255;
    request->force = 255;
    // Call the gripper service asynchronously
    auto future = gripper_client->async_send_request(request);
    // Wait for the responses
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown
  rclcpp::shutdown();
  spinner.join();
  return 0;
}