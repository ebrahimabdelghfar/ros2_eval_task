#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <random>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "ros2_eval_task/gazebo_utils_client.hpp"

class ModelSpawnerNode : public rclcpp::Node
{
public:
    ModelSpawnerNode() : Node("model_spawner_node")
    {
        // Initialize the Gazebo client using a separate node for its services
        rclcpp::NodeOptions options;
        gazebo_client_node_ = std::make_shared<ros2_eval_task::GazeboUtilsClient>(options);

        // Get package path to find models and create images directory
        package_path_ = ament_index_cpp::get_package_share_directory("ros2_eval_task");
        std::string models_path = package_path_ + "/models/";
        images_path_ = package_path_ + "/images/";

        // Create images directory if it doesn't exist
        std::string create_dir_cmd = "mkdir -p " + images_path_;
        std::system(create_dir_cmd.c_str());

        // List of models to spawn
        model_files_ = {
            models_path + "battery_9v_leader/model.sdf",
            models_path + "battery_energizer/model.sdf",
            models_path + "battery_varita/model.sdf",
            models_path + "lipo_battery/model.sdf"};
        model_base_names_ = {
            "battery_9v_leader",
            "battery_energizer",
            "battery_varita",
            "lipo_battery"};

        // Create image subscriber for one-time captures
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&ModelSpawnerNode::image_callback, this, std::placeholders::_1));

        // Create a timer that fires every 5 seconds
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&ModelSpawnerNode::timer_callback, this));

        // Create a one-shot timer for delayed image capture (will be set up when needed)
        // This timer is not started here, but created when model spawning succeeds
        // Setup random number generation for position
        std::random_device rd;
        rng_ = std::mt19937(rd());
        x_dist_ = std::uniform_real_distribution<double>(-0.1, 0.1);
        y_dist_ = std::uniform_real_distribution<double>(-0.1, 0.1);

        RCLCPP_INFO(this->get_logger(), "Model Spawner Node has started.");
        RCLCPP_INFO(this->get_logger(), "Images will be saved to: %s", images_path_.c_str());
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Only save image if we're waiting for one (after successful model spawn)
        if (!capture_next_image_)
        {
            return;
        }
        else
        {
            rclcpp::sleep_for(std::chrono::milliseconds(2000));
            // Convert ROS image to OpenCV image
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            // Generate filename with model name and timestamp
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch()) %
                      1000;

            std::stringstream ss;
            ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
            ss << "_" << std::setfill('0') << std::setw(3) << ms.count();
            std::string filename = images_path_ + model_base_names_[current_model_index_] + "_" + ss.str() + ".png";

            // Save image as PNG
            if (cv::imwrite(filename, cv_ptr->image))
            {
                RCLCPP_INFO(this->get_logger(), "Image saved after spawning '%s': %s",
                            model_base_names_[current_model_index_].c_str(), filename.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to save image: %s", filename.c_str());
            }

            // Reset flag after capturing image
            capture_next_image_ = false;
        }
    }
    void timer_callback()
    {
        // 1. Delete the previous model if it exists
        if (!last_model_name_.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Deleting model: %s", last_model_name_.c_str());
            gazebo_client_node_->delete_model(last_model_name_);
        }
        // 2. Select the next model to spawn
        current_model_index_ = (current_model_index_ + 1) % model_files_.size();
        const std::string &model_path = model_files_[current_model_index_];
        const std::string &model_name = model_base_names_[current_model_index_];
        // 3. Read model XML content from file
        std::ifstream file(model_path);
        if (!file.is_open())
        {
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

        bool spawn_success = gazebo_client_node_->spawn_model(last_model_name_, model_xml, pose);
        // If spawning was successful, trigger image capture
        if (spawn_success)
        {
            RCLCPP_INFO(this->get_logger(), "Model spawned successfully, capturing image...");
            capture_next_image_ = true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to spawn model '%s'", last_model_name_.c_str());
        }
    }

    // Member variables
    cv_bridge::CvImagePtr cv_ptr;
    std::vector<std::string> model_files_;
    std::vector<std::string> model_base_names_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<ros2_eval_task::GazeboUtilsClient> gazebo_client_node_;
    std::string last_model_name_ = "";
    std::mt19937 rng_;
    std::uniform_real_distribution<double> x_dist_;
    std::uniform_real_distribution<double> y_dist_;
    size_t current_model_index_ = -1;

    // Image processing members
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    std::string package_path_;
    std::string images_path_;
    bool capture_next_image_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Using SingleThreadedExecutor to handle both the spawner node's timer and the client node's services
    rclcpp::executors::SingleThreadedExecutor executor;
    auto spawner_node = std::make_shared<ModelSpawnerNode>();
    executor.add_node(spawner_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}