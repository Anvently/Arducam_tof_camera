#include "Arducam_tof_publisher.hpp"

void ArducamTofPublisher::initializeParameters()
{
	// Declare parameters with default values
	this->declare_parameter("publish_rate_ms", publish_rate_ms_);
	this->declare_parameter("range_max", range_max_);
	this->declare_parameter("jpeg_quality", jpeg_quality_);
	this->declare_parameter("topic_name", topic_name_);
	this->declare_parameter("frame_id", frame_id_);
	
	// Get parameter values
	publish_rate_ms_ = this->get_parameter("publish_rate_ms").as_int();
	range_max_ = this->get_parameter("range_max").as_int();
	jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();
	topic_name_ = this->get_parameter("topic_name").as_string();
	frame_id_ = this->get_parameter("frame_id").as_string();
	
	// Validate parameters
	if (publish_rate_ms_ < 1) {
		RCLCPP_WARN(this->get_logger(), "Invalid publish_rate_ms: %d, using default: 30", publish_rate_ms_);
		publish_rate_ms_ = 30;
	}
	
	if (range_max_ < 1000 || range_max_ > 10000) {
		RCLCPP_WARN(this->get_logger(), "Invalid range_max: %d, using default: 4000", range_max_);
		range_max_ = 4000;
	}
	
	if (jpeg_quality_ < 1 || jpeg_quality_ > 100) {
		RCLCPP_WARN(this->get_logger(), "Invalid jpeg_quality: %d, using default: 95", jpeg_quality_);
		jpeg_quality_ = 95;
	}
	
	// Set up parameter callback for dynamic reconfiguration
	params_callback_handle_ = this->add_on_set_parameters_callback(
		[this](const std::vector<rclcpp::Parameter> & parameters) -> rcl_interfaces::msg::SetParametersResult {
			rcl_interfaces::msg::SetParametersResult result;
			result.successful = true;
			
			for (const auto & param : parameters) {
				if (param.get_name() == "publish_rate_ms") {
					int value = param.as_int();
					if (value < 1) {
						result.successful = false;
						result.reason = "publish_rate_ms must be greater than 0";
					} else {
						publish_rate_ms_ = value;
						// Recreate timer with new rate
						timer_ = this->create_wall_timer(
							std::chrono::milliseconds(publish_rate_ms_), 
							std::bind(&ArducamTofPublisher::publishFrame, this));
						RCLCPP_INFO(this->get_logger(), "Updated publish_rate_ms to %d", publish_rate_ms_);
					}
				} else if (param.get_name() == "range_max") {
					int value = param.as_int();
					if (value < 1000 || value > 10000) {
						result.successful = false;
						result.reason = "range_max must be between 1000 and 10000";
					} else {
						range_max_ = value;
						if (camera_initialized_) {
							tof_camera_.setControl(Arducam::Control::RANGE, range_max_);
							tof_camera_.getControl(Arducam::Control::RANGE, &max_range_);
							RCLCPP_INFO(this->get_logger(), "Updated range_max to %d", max_range_);
						}
					}
				} else if (param.get_name() == "jpeg_quality") {
					int value = param.as_int();
					if (value < 1 || value > 100) {
						result.successful = false;
						result.reason = "jpeg_quality must be between 1 and 100";
					} else {
						jpeg_quality_ = value;
						RCLCPP_INFO(this->get_logger(), "Updated jpeg_quality to %d", jpeg_quality_);
					}
				} else if (param.get_name() == "frame_id") {
					frame_id_ = param.as_string();
					RCLCPP_INFO(this->get_logger(), "Updated frame_id to %s", frame_id_.c_str());
				}
				// Note: topic_name cannot be changed dynamically
			}
			
			return result;
		});
	
	RCLCPP_INFO(this->get_logger(), "Parameters initialized:");
	RCLCPP_INFO(this->get_logger(), "  publish_rate_ms: %d", publish_rate_ms_);
	RCLCPP_INFO(this->get_logger(), "  range_max: %d", range_max_);
	RCLCPP_INFO(this->get_logger(), "  jpeg_quality: %d", jpeg_quality_);
	RCLCPP_INFO(this->get_logger(), "  topic_name: %s", topic_name_.c_str());
	RCLCPP_INFO(this->get_logger(), "  frame_id: %s", frame_id_.c_str());
}#include "arducam_tof_publisher.hpp"

ArducamTofPublisher::ArducamTofPublisher() 
: Node("arducam_tof_publisher"),
camera_initialized_(false),
max_range_(4000),
frame_count_(0),
time_begin_(std::chrono::high_resolution_clock::now()),
publish_rate_ms_(30),
range_max_(4000),
jpeg_quality_(95),
topic_name_("arducam_tof/image/compressed"),
frame_id_("arducam_tof_frame")
{
	// Initialize ROS2 parameters
	initializeParameters();
	
	// Create publisher for compressed images
	publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
		topic_name_, 10);
	
	// Initialize the camera
	if (tof_camera_.open(Arducam::Connection::CSI)) {
		RCLCPP_ERROR(this->get_logger(), "Failed to initialize Arducam TOF camera");
		return;
	}
	
	if (tof_camera_.start(Arducam::FrameType::RAW_FRAME)) {
		RCLCPP_ERROR(this->get_logger(), "Failed to start Arducam TOF camera");
		return;
	}
	
	// Set camera range based on parameter
	tof_camera_.setControl(Arducam::Control::RANGE, range_max_);
	
	// Get actual camera range (in case setting failed)
	tof_camera_.getControl(Arducam::Control::RANGE, &max_range_);
	
	// Get camera info
	Arducam::CameraInfo info = tof_camera_.getCameraInfo();
	RCLCPP_INFO(this->get_logger(), 
				"Opened camera with resolution %dx%d and range %d", 
				info.width, info.height, max_range_);
	
	camera_initialized_ = true;
	
	// Create timer for frame capture and publishing
	timer_ = this->create_wall_timer(
		std::chrono::milliseconds(publish_rate_ms_), 
		std::bind(&ArducamTofPublisher::publishFrame, this));
		
	RCLCPP_INFO(this->get_logger(), "Arducam TOF publisher initialized");
}

ArducamTofPublisher::~ArducamTofPublisher()
{
	cleanup();
}

void ArducamTofPublisher::publishFrame()
{
	if (!camera_initialized_) {
		return;
	}
	
	// Request frame from camera
	Arducam::ArducamFrameBuffer* frame = tof_camera_.requestFrame(2000);
	if (frame == nullptr) {
		RCLCPP_WARN(this->get_logger(), "Failed to capture frame from TOF camera");
		return;
	}
	
	Arducam::FrameFormat format;
	frame->getFormat(Arducam::FrameType::RAW_FRAME, format);
	
	int16_t* raw_ptr = (int16_t*)frame->getData(Arducam::FrameType::RAW_FRAME);
	if (raw_ptr == nullptr) {
		tof_camera_.releaseFrame(frame);
		RCLCPP_WARN(this->get_logger(), "Failed to get raw frame data");
		return;
	}
	
	// Convert raw data to OpenCV Mat
	cv::Mat raw_frame(format.width, format.height, CV_16S, raw_ptr);
	cv::Mat result_frame;
	raw_frame.convertTo(result_frame, CV_8U, 1. / (1 << 4), 0);
	
	// Create compressed image message
	sensor_msgs::msg::CompressedImage compressed_img;
	compressed_img.header.stamp = this->now();
	compressed_img.header.frame_id = frame_id_;
	compressed_img.format = "jpeg";
	
	// Compress the image using OpenCV
	std::vector<uchar> compressed_data;
	std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
	cv::imencode(".jpg", result_frame, compressed_data, compression_params);
	
	// Copy compressed data to message
	compressed_img.data = compressed_data;
	
	// Publish the compressed image
	publisher_->publish(compressed_img);
	
	// Display FPS info
	displayFps();
	
	// Release frame
	tof_camera_.releaseFrame(frame);
}

void ArducamTofPublisher::displayFps()
{
	using namespace std::chrono;
	
	frame_count_++;
	auto time_end = high_resolution_clock::now();
	auto duration_ms = duration_cast<milliseconds>(time_end - time_begin_).count();
	
	if (duration_ms >= 1000) {
		RCLCPP_INFO(this->get_logger(), "FPS: %d", frame_count_);
		frame_count_ = 0;
		time_begin_ = time_end;
	}
}

void ArducamTofPublisher::cleanup()
{
	if (camera_initialized_) {
		if (tof_camera_.stop()) {
			RCLCPP_ERROR(this->get_logger(), "Failed to stop camera");
		}
		
		if (tof_camera_.close()) {
			RCLCPP_ERROR(this->get_logger(), "Failed to close camera");
		}
		
		camera_initialized_ = false;
	}
	
	// Remove parameter callback
	if (params_callback_handle_) {
		this->remove_on_set_parameters_callback(params_callback_handle_);
		params_callback_handle_.reset();
	}
}

// Main function to start the node
int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ArducamTofPublisher>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}