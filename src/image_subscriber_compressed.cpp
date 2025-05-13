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
    : Node("image_subscriber_qos")
    {
        auto image_qos = rclcpp::SystemDefaultsQoS();

        subscription_=this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "image_compressed", image_qos, std::bind(&ImageSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
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

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}
