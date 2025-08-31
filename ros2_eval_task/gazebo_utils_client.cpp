#include "ros2_eval_task/gazebo_utils_client.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace ros2_eval_task
{

GazeboUtilsClient::GazeboUtilsClient()
: rclcpp::Node("gazebo_utils_client_node")
{
  // Create a client for the spawn service
  spawn_client_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
  // Create a client for the delete service
  delete_client_ = this->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");

  RCLCPP_INFO(this->get_logger(), "GazeboUtilsClient node initialized.");
}

bool GazeboUtilsClient::spawn_model(
  const std::string & model_name,
  const std::string & model_xml,
  const geometry_msgs::msg::Pose & pose)
{
  // Wait for the service to be available
  if (!spawn_client_->wait_for_service(2s)) {
    RCLCPP_ERROR(this->get_logger(), "Spawn service not available.");
    return false;
  }

  // Create the request
  auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
  request->name = model_name;
  request->xml = model_xml;
  request->initial_pose = pose;

  // Send the request asynchronously
  auto future = spawn_client_->async_send_request(request);

  // Wait for the result
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call spawn service.");
    return false;
  }

  // Check the response
  auto result = future.get();
  if (result->success) {
    RCLCPP_INFO(this->get_logger(), "Successfully spawned model: %s", model_name.c_str());
  } else {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to spawn model: %s. Status: %s",
      model_name.c_str(), result->status_message.c_str());
  }

  return result->success;
}

bool GazeboUtilsClient::delete_model(const std::string & model_name)
{
  // Wait for the service to be available
  if (!delete_client_->wait_for_service(2s)) {
    RCLCPP_ERROR(this->get_logger(), "Delete service not available.");
    return false;
  }

  // Create the request
  auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
  request->name = model_name;

  // Send the request asynchronously
  auto future = delete_client_->async_send_request(request);

  // Wait for the result
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call delete service.");
    return false;
  }

  // Check the response
  auto result = future.get();
  if (result->success) {
    RCLCPP_INFO(this->get_logger(), "Successfully deleted model: %s", model_name.c_str());
  } else {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to delete model: %s. Status: %s",
      model_name.c_str(), result->status_message.c_str());
  }

  return result->success;
}

}