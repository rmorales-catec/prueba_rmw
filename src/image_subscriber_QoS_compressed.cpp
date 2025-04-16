#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/qos.hpp>
#include <opencv2/opencv.hpp>

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber()
    : Node("image_subscriber_qos"), count_(0)
    {
        rclcpp::QoS image_qos(1000);
        // image_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT); 
        image_qos.history(RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT);
        image_qos.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
        image_qos.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);

        subscription_=this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "image_compressed", image_qos, std::bind(&ImageSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        auto message = std_msgs::msg::String();
        message.data = std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "'%s'", message.data.c_str());

        try{
            // Convertir los datos del mensaje a un cv::Mat
            std::vector<uint8_t> data(msg->data.begin(), msg->data.end());
            cv::Mat image = cv::imdecode(data, cv::IMREAD_COLOR);

            if (!image.empty())
            {
            cv::imshow("Imagen Recibida", image);
            cv::waitKey(1);
            }
        }
        catch(const cv_bridge::Exception& excep){
            RCLCPP_ERROR(this->get_logger(), "Error al convertir la imagen: %s", excep.what());
        }
    }

    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_1;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_1;
    int count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}
