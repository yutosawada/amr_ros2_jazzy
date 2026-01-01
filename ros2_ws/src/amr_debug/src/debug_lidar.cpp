#include <memory>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class DebugLidar : public rclcpp::Node
{
public:
  DebugLidar()
  : Node("debug_lidar")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&DebugLidar::topic_callback, this, _1));
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    rclcpp::Time now = this->get_clock()->now();
    rclcpp::Time scan_time(msg->header.stamp);
    double diff = (now - scan_time).seconds();

    RCLCPP_INFO(this->get_logger(), "==========================================");
    RCLCPP_INFO(this->get_logger(), "Current ROS Time: %f", now.seconds());
    RCLCPP_INFO(this->get_logger(), "Scan Header Time: %f", scan_time.seconds());
    RCLCPP_INFO(this->get_logger(), "Difference: %.6f s", diff);

    std::string target_frame = "base_link";
    std::string source_frame = msg->header.frame_id;

    try {
      if (tf_buffer_->canTransform(target_frame, source_frame, rclcpp::Time(0))) {
        geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
          target_frame, source_frame, rclcpp::Time(0));
        
        rclcpp::Time tf_time(t.header.stamp);
        double tf_diff = (now - tf_time).seconds();
        
        RCLCPP_INFO(this->get_logger(), "Latest TF Time: %f", tf_time.seconds());
        RCLCPP_INFO(this->get_logger(), "TF Age: %.6f s", tf_diff);
      } else {
        RCLCPP_WARN(this->get_logger(), "Cannot transform from %s to %s", source_frame.c_str(), target_frame.c_str());
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "TF Error: %s", ex.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DebugLidar>());
  rclcpp::shutdown();
  return 0;
}
