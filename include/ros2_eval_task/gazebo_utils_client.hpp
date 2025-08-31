#ifndef GAZEBO_UTILS_CLIENT_HPP_
#define GAZEBO_UTILS_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include "geometry_msgs/msg/pose.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"

namespace sigma
{

class GazeboUtilsClient
{
public:
    /**
     * @brief Construct a new Gazebo Utils Client object
     * @param node A pointer to the ROS 2 node that will own this client.
     */
    explicit GazeboUtilsClient(rclcpp::Node* node); // CHANGED: Now accepts a raw pointer

    /**
     * @brief Spawns a model in Gazebo.
     * @param model_name The name for the spawned model.
     * @param xml The XML (SDF/URDF) content of the model.
     * @param pose The pose at which to spawn the model.
     * @return true if spawning was successful, false otherwise.
     */
    bool spawn_model(
        const std::string &model_name,
        const std::string &xml,
        const geometry_msgs::msg::Pose &pose);

    /**
     * @brief Deletes a model from Gazebo.
     * @param model_name The name of the model to delete.
     * @return true if deletion was successful, false otherwise.
     */
    bool delete_model(const std::string &model_name);

private:
    rclcpp::Node* node_; // CHANGED: Member is now a raw pointer
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_;
};

} // namespace sigma

#endif // GAZEBO_UTILS_CLIENT_HPP_