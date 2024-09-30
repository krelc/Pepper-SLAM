#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class CameraFaceDetection : public rclcpp::Node
{
public:
	CameraFaceDetection()
		: Node("camera_face_detection")
	{
		// Create a publisher to publish the image with detected faces
		image_publisher_ = create_publisher<sensor_msgs::msg::Image>("detected_faces", 10);

		// Create a subscriber to the /camera/front/image_raw topic
		image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
			"/camera/bottom/image_raw",
			10,
			std::bind(&CameraFaceDetection::imageCallback, this, std::placeholders::_1));

		// Load the face cascade classifier
		if (!face_cascade_.load("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml")) {
			RCLCPP_ERROR(this->get_logger(), "Error loading face cascade classifier");
		}
	}

private:
	void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
	{
		// Convert ROS image message to OpenCV image
		cv::Mat frame;
		try {
			frame = cv_bridge::toCvShare(msg, "bgr8")->image;
		} catch (cv_bridge::Exception& e) {
			RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
			return;
		}

		// Convert the frame to grayscale for face detection
		cv::Mat gray_image;
		cv::cvtColor(frame, gray_image, cv::COLOR_BGR2GRAY);

		// Detect faces in the grayscale image
		std::vector<cv::Rect> faces;
		face_cascade_.detectMultiScale(gray_image, faces);

		// Draw rectangles around detected faces
		for (const auto& face : faces) {
			cv::rectangle(frame, face, cv::Scalar(255, 0, 0), 2);
		}

		// Convert the modified frame back to a ROS image message
		auto msg_out = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

		// Publish the modified image
		image_publisher_->publish(*msg_out);
	}

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
	cv::CascadeClassifier face_cascade_;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<CameraFaceDetection>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}