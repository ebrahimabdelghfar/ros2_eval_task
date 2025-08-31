#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <random>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ros2_eval_task/gazebo_utils_client.hpp"

class ModelSpawnerNode : public rclcpp::Node
{
public:
  ModelSpawnerNode() : Node("model_spawner_node")
  {
    // Initialize the Gazebo client using a separate node for its services
    rclcpp::NodeOptions options;
    gazebo_client_node_ = std::make_shared<ros2_eval_task::GazeboUtilsClient>(options);
    // Get package path to find models
    std::string package_path = ament_index_cpp::get_package_share_directory("ros2_eval_task");
    std::string models_path = package_path + "/models/";
    // List of models to spawn
    model_files_ = {
        models_path + "battery_9v_leader/model.sdf",
        models_path + "battery_energizer/model.sdf",
        models_path + "battery_varita/model.sdf",
        models_path + "lipo_battery/model.sdf"
    };
    model_base_names_ = {
        "battery_9v_leader",
        "battery_energizer",
        "battery_varita",
        "lipo_battery"
    };
    // Create a timer that fires every 5 seconds
    timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&ModelSpawnerNode::timer_callback, this));
    // Setup random number generation for position
    std::random_device rd;
    rng_ = std::mt19937(rd());
    x_dist_ = std::uniform_real_distribution<double>(-0.21, 0.21);
    y_dist_ = std::uniform_real_distribution<double>(-0.43, 0.43);


    RCLCPP_INFO(this->get_logger(), "Model Spawner Node has started.");
  }

private:
  void timer_callback()
  {
    // 1. Delete the previous model if it exists
    if (!last_model_name_.empty()) {
      RCLCPP_INFO(this->get_logger(), "Deleting model: %s", last_model_name_.c_str());
      gazebo_client_node_->delete_model(last_model_name_);
    }
    // 2. Select the next model to spawn
    current_model_index_ = (current_model_index_ + 1) % model_files_.size();
    const std::string& model_path = model_files_[current_model_index_];
    const std::string& model_name = model_base_names_[current_model_index_];
    // 3. Read model XML content from file
    std::ifstream file(model_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open model file: %s", model_path.c_str());
        return;
    }
    std::string model_xml((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();
    // 4. Generate a random pose
    last_model_name_ = model_name;
    geometry_msgs::msg::Pose pose;
    pose.position.x = x_dist_(rng_);
    pose.position.y = y_dist_(rng_);
    pose.position.z = 1.1;
    pose.orientation.w = 1.0; // No rotation
    // 3. Spawn the new model
    RCLCPP_INFO(this->get_logger(), "Spawning model '%s' at [x: %.2f, y: %.2f]",
    last_model_name_.c_str(), pose.position.x, pose.position.y);
    gazebo_client_node_->spawn_model(last_model_name_, model_xml, pose);
  }

  // Member variables
  std::vector<std::string> model_files_;
  std::vector<std::string> model_base_names_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<ros2_eval_task::GazeboUtilsClient> gazebo_client_node_;
  std::string last_model_name_ = "";
  std::mt19937 rng_;
  std::uniform_real_distribution<double> x_dist_;
  std::uniform_real_distribution<double> y_dist_;
  size_t current_model_index_ = -1;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // Using MultiThreadedExecutor to handle both the spawner node's timer and the client node's services
  rclcpp::executors::MultiThreadedExecutor executor;
  auto spawner_node = std::make_shared<ModelSpawnerNode>();
  executor.add_node(spawner_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}