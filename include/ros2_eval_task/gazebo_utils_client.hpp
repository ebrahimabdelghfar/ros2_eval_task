#ifndef GAZEBO_UTILS_CLIENT_HPP_
#define GAZEBO_UTILS_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include <string>

namespace ros2_eval_task
{

class GazeboUtilsClient : public rclcpp::Node
{
public:
  // Constructor
  GazeboUtilsClient();

  /**
   * @brief Spawns a model in the Gazebo simulation.
   * @param model_name The name for the new model.
   * @param model_xml The SDF or URDF content of the model as a string.
   * @param pose The initial pose (position and orientation) of the model.
   * @return True if the model was spawned successfully, false otherwise.
   */
  bool spawn_model(
    const std::string & model_name,
    const std::string & model_xml,
    const geometry_msgs::msg::Pose & pose);

  /**
   * @brief Deletes a model from the Gazebo simulation.
   * @param model_name The name of the model to delete.
   * @return True if the model was deleted successfully, false otherwise.
   */
  bool delete_model(const std::string & model_name);

private:
  // Service clients
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_;
  rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_;
};

}  // namespace ros2_eval_task

#endif  // GAZEBO_UTILS_CLIENT_HPP_