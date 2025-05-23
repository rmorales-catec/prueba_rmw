#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/qos.hpp>
#include <string>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


class ImagePublisher : public rclcpp::Node
{
public: 
  ImagePublisher()
  : Node("image_publisher_qos")
  {
    auto image_qos = rclcpp::SystemDefaultsQoS();
      
    //Definimos el publisher, declaramos el tipo de mensaje, el topic en el que vamos a publicar
    publisher_=this->create_publisher<sensor_msgs::msg::CompressedImage>("image_compressed", image_qos); 
    capture_.open(0);
    if (!capture_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "No se pudo abrir la cámara.");
    }
    capture_.set(cv::CAP_PROP_FPS, 30);

    //Creamos un timer para enviar imágenes a ~30fps
    timer_=this->create_wall_timer(
        std::chrono::milliseconds(33), std::bind(&ImagePublisher::timer_callback, this));
  }

private:
  //Creamos una función con la que enviaremos la imagen al topic /image
  void timer_callback()
  {
    cv::Mat frame;  //variable donde vamos a guardar la imagen. Es una matriz
    capture_ >> frame; //Guardamos la imagen que hemos capturado en capture_

    if (frame.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Error al capturar la imagen de la cámara.");
      return;
    }

    try { 
      // Comprimir imagen en formato JPEG
      std::vector<uint8_t> buffer;
      std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90}; 
      cv::imencode(".jpg", frame, buffer, compression_params);

      // Crear mensaje CompressedImage
      auto ros_image = sensor_msgs::msg::CompressedImage();
      ros_image.header.stamp = this->get_clock()->now();
      ros_image.format = "jpeg";
      ros_image.data = buffer; 

      // Publicar la imagen
      publisher_->publish(ros_image);

    } 
    catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error al convertir la imagen: %s", e.what());
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
  cv::VideoCapture capture_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}
