#ifndef ARDUCAM_TOF_PUBLISHER_HPP
#define ARDUCAM_TOF_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <memory>
#include "ArducamTOFCamera.hpp"

using namespace std::chrono_literals;

class ArducamTofPublisher : public rclcpp::Node
{
public:
	ArducamTofPublisher();
	~ArducamTofPublisher();

private:
	void initializeParameters();
	void publishFrame();
	void displayFps();
	void cleanup();
	
	// ROS2 Publisher
	rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
	
	// Parameters
	int publish_rate_ms_;
	int range_max_;
	int jpeg_quality_;
	std::string topic_name_;
	std::string frame_id_;
	
	// Parameter callback handle
	OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
	
	// Arducam TOF Camera
	Arducam::ArducamTOFCamera tof_camera_;
	bool camera_initialized_;
	int max_range_;
	
	// FPS calculation
	int frame_count_;
	std::chrono::high_resolution_clock::time_point time_begin_;
};

#endif // ARDUCAM_TOF_PUBLISHER_HPP