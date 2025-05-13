#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/qos.hpp>

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber()
    : Node("image_subscriber_qos")
    {
        auto image_qos = rclcpp::SystemDefaultsQoS();

        subscription_=this->create_subscription<sensor_msgs::msg::Image>(
            "image", image_qos, std::bind(&ImageSubscriber::topic_callback, this, std::placeholders::_1));
        
            // Inicializar ventana de OpenCV una sola vez
        //cv::namedWindow("Received Image", cv::WINDOW_AUTOSIZE);
    }

private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {

        try{
            // Convertir el mensaje ROS (sensor_msgs::msg::Image) a OpenCV (cv::Mat)
            cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, "bgr8");
            // Mostrar la imagen usando OpenCV
            cv::imshow("Received Image", cv_image->image);
            cv::waitKey(10);  // Para permitir que OpenCV actualice la ventana
        }
        catch(const cv_bridge::Exception& excep){
            RCLCPP_ERROR(this->get_logger(), "Error al convertir la imagen: %s", excep.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}
