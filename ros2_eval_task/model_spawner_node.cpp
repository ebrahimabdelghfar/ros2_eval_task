#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <random>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ros2_eval_task/gazebo_utils_client.hpp"

using namespace std::chrono_literals;

class ModelSpawnerNode : public rclcpp::Node
{
public:
    ModelSpawnerNode() : Node("model_spawner_node")
    {
        // Initialize the Gazebo client by passing a pointer to this node
        gazebo_client_ = std::make_unique<sigma::GazeboUtilsClient>(this); // CHANGED: Pass 'this'

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
        
        model_base_names_ = {"battery_9v_leader", "battery_energizer", "battery_varita", "lipo_battery"};

        // Setup random number generation for position
        std::random_device rd;
        rng_ = std::mt19937(rd());
        x_dist_ = std::uniform_real_distribution<double>(-0.21, 0.21);
        y_dist_ = std::uniform_real_distribution<double>(-0.43, 0.43);

        // Create a timer for spawning and deleting models every 5 seconds
        timer_ = this->create_wall_timer(
            5s, std::bind(&ModelSpawnerNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // 1. Delete the previous model if it exists
        if (!last_spawned_model_name_.empty()) {
            gazebo_client_->delete_model(last_spawned_model_name_);
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
        geometry_msgs::msg::Pose pose;
        pose.position.x = x_dist_(rng_);
        pose.position.y = y_dist_(rng_);
        pose.position.z = 1.1;
        pose.orientation.w = 1.0; // No rotation

        RCLCPP_INFO(this->get_logger(), "Attempting to spawn '%s' at [x: %.2f, y: %.2f, z: %.2f]",
            model_name.c_str(), pose.position.x, pose.position.y, pose.position.z);
        
        // 5. Spawn the new model
        if (gazebo_client_->spawn_model(model_name, model_xml, pose)) {
            last_spawned_model_name_ = model_name;
        } else {
            last_spawned_model_name_.clear();
        }
    }

    // Member variables
    std::unique_ptr<sigma::GazeboUtilsClient> gazebo_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> model_files_;
    std::vector<std::string> model_base_names_;
    std::string last_spawned_model_name_;
    size_t current_model_index_ = -1;

    // Random number generation
    std::mt19937 rng_;
    std::uniform_real_distribution<double> x_dist_;
    std::uniform_real_distribution<double> y_dist_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModelSpawnerNode>());
    rclcpp::shutdown();
    return 0;
}