#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"  // For converting ROS images to OpenCV
#include "opencv2/imgproc.hpp"    // For OpenCV image processing

class ImageConversionNode : public rclcpp::Node {
public:
    ImageConversionNode()
    : Node("image_conversion"), mode_(false) {
        // Declare the input and output topic parameters
        this->declare_parameter<std::string>("input_topic", "/usb_cam/image_raw");
        this->declare_parameter<std::string>("output_topic", "/converted_image");

        // Create a service to toggle the mode
        service_ = this->create_service<std_srvs::srv::SetBool>(
            "toggle_mode",
            std::bind(&ImageConversionNode::toggleMode, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Subscribe to the input image topic
        std::string input_topic = this->get_parameter("input_topic").as_string();
        
        subscription_ = image_transport::create_subscription(
            this,
            input_topic,  // Use the parameterized input topic
            [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) { 
                this->processImage(msg); 
            },
            "raw"
        );

        // Publish to the output image topic
        std::string output_topic = this->get_parameter("output_topic").as_string();
        publisher_ = image_transport::create_publisher(this, output_topic);

        RCLCPP_INFO(this->get_logger(), "ImageConversionNode initialized");
    }

private:
    void toggleMode(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response
    ) {
        mode_ = request->data;
        response->success = true;
        response->message = mode_ ? "Mode activated" : "Mode deactivated";
        RCLCPP_INFO(this->get_logger(), response->message.c_str());
    }

    void processImage(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        // Convert ROS image message to OpenCV format using cv_bridge
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Convert to grayscale using OpenCV
        if (mode_) {
        cv::Mat gray_image;
        cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

        // Convert the grayscale image back to a ROS message
        sensor_msgs::msg::Image::SharedPtr output_msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "mono8", gray_image).toImageMsg();

        // Publish the grayscale image
        publisher_.publish(output_msg);}
        
        else {
        publisher_.publish(msg);
        }

        if (mode_) {
        RCLCPP_INFO(this->get_logger(), "Grayscale image processed and published.");
               }       
        else {
        RCLCPP_INFO(this->get_logger(), "Color image directly published.");
                }

    }

    bool mode_;  // The mode (activated/deactivated)
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;  // Service to toggle mode
    image_transport::Subscriber subscription_;  // Subscriber to input topic
    image_transport::Publisher publisher_;  // Publisher to output topic
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);  // Initialize ROS 2
    rclcpp::spin(std::make_shared<ImageConversionNode>());  // Spin the node
    rclcpp::shutdown();  // Shutdown ROS 2
    return 0;
}

