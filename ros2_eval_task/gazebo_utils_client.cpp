#include "ros2_eval_task/gazebo_utils_client.hpp"

using namespace std::chrono_literals;

namespace sigma
{

// CHANGED: Constructor now accepts a raw pointer
GazeboUtilsClient::GazeboUtilsClient(rclcpp::Node* node) : node_(node)
{
    // Note: The document states to use '/gazebo/factory/get_entity_state'.
    // This appears to be a typo, as that service gets state, not spawns entities.
    // The correct service for spawning is '/spawn_entity'.
    spawn_client_ = node_->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

    delete_client_ = node_->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
}

bool GazeboUtilsClient::spawn_model(
    const std::string &model_name,
    const std::string &xml,
    const geometry_msgs::msg::Pose &pose)
{
    if (!spawn_client_->wait_for_service(2.0s)) {
        RCLCPP_ERROR(node_->get_logger(), "Spawn service not available.");
        return false;
    }

    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = model_name;
    request->xml = xml;
    request->initial_pose = pose;

    auto future = spawn_client_->async_send_request(request);
    auto status = future.wait_for(2.0s);

    if (status != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call spawn service");
        return false;
    }

    auto result = future.get();
    if (!result->success) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to spawn model '%s': %s", model_name.c_str(), result->status_message.c_str());
    } else {
         RCLCPP_INFO(node_->get_logger(), "Successfully spawned model '%s'", model_name.c_str());
    }
    return result->success;
}

bool GazeboUtilsClient::delete_model(const std::string &model_name)
{
    if (!delete_client_->wait_for_service(2.0s)) {
        RCLCPP_ERROR(node_->get_logger(), "Delete service not available.");
        return false;
    }

    auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    request->name = model_name;

    auto future = delete_client_->async_send_request(request);
    auto status = future.wait_for(2.0s);

    if (status != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call delete service");
        return false;
    }

    auto result = future.get();
     if (!result->success) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to delete model '%s': %s", model_name.c_str(), result->status_message.c_str());
    } else {
         RCLCPP_INFO(node_->get_logger(), "Successfully deleted model '%s'", model_name.c_str());
    }
    return result->success;
}

} // namespace sigma