#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LaserDataTransform : public rclcpp::Node {
public:
    LaserDataTransform() : Node("laser_data_transform") {
        tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
        tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Increase the broadcast rate to 10 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&LaserDataTransform::broadcastTransform, this)
        );

        // Subscribe to the /scan topic
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 100, std::bind(&LaserDataTransform::scanCallback, this, std::placeholders::_1)
        );

        // Publisher for the modified scan data
        modified_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 100);
    }

private:
    void broadcastTransform() {
        geometry_msgs::msg::TransformStamped laserTransform;
        laserTransform.header.stamp = this->get_clock()->now();
        laserTransform.header.frame_id = "base_footprint"; // Parent frame
        laserTransform.child_frame_id = "laser_frame"; // Child frame
        laserTransform.transform.translation.x = 0.0; // Adjust the position relative to base_footprint
        laserTransform.transform.translation.y = 0.0;
        laserTransform.transform.translation.z = 0.1; // Adjust the height if needed
        laserTransform.transform.rotation.x = 0.0;
        laserTransform.transform.rotation.y = 0.0;
        laserTransform.transform.rotation.z = 0.0;
        laserTransform.transform.rotation.w = 1.0;

        // Broadcast the new transform for the laser
        tfBroadcaster_->sendTransform(laserTransform);
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // auto modified_scan = *msg;

        // // Calculate the angle range for the 120-degree cone
        // double angle_min = -60.0 * M_PI / 180.0; // -60 degrees in radians
        // double angle_max = 60.0 * M_PI / 180.0;  // 60 degrees in radians

        // // Iterate through the scan data and filter out points outside the 120-degree cone
        // for (size_t i = 0; i < msg->ranges.size(); ++i) {
        //     double angle = msg->angle_min + i * msg->angle_increment;
        //     if (angle < angle_min || angle > angle_max) {
        //         modified_scan.ranges[i] = std::numeric_limits<float>::infinity(); // Set to infinity to ignore
        //     }
        // }

        // // Ensure the modified scan has the same timestamp as the original scan
        // modified_scan.header.stamp = msg->header.stamp;

        // // Log the timestamps for debugging
        // // RCLCPP_INFO(this->get_logger(), "Scan timestamp: %f", msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9);
        // // RCLCPP_INFO(this->get_logger(), "Transform timestamp: %f", this->get_clock()->now().seconds());

        // // Publish the modified scan data to the same topic
        // modified_scan_pub_->publish(modified_scan);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr modified_scan_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserDataTransform>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}