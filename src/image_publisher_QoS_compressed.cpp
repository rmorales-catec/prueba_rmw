#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/qos.hpp>
#include <string>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

/* Declaramos la clase que va a publicar la imagen en el topic /image */

class ImagePublisher : public rclcpp::Node
{
public: 
  ImagePublisher()
  : Node("image_publisher_qos"), count_(0), count_2(0)  //Declaramos el nombre del nodo e inicializamos un contador
  {
    // Configuración de QoS de los dos topics
    rclcpp::QoS image_qos(1000);
    // image_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    image_qos.history(RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT);
    image_qos.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
    image_qos.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
      
    //Definimos el publisher, declaramos el tipo de mensaje, el topic en el que vamos a publicar y el tamaño de la cola 
    publisher_=this->create_publisher<sensor_msgs::msg::CompressedImage>("image_compressed", image_qos); 
    capture_.open(0);  // Para usar la cámara por defecto
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
    auto message = std_msgs::msg::String();
    message.data = std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "'%s'", message.data.c_str());
    // publisher_1->publish(message);


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

      RCLCPP_INFO(this->get_logger(), "Publicando imagen #%d", count_2++);
    } 
    catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error al convertir la imagen: %s", e.what());
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
  cv::VideoCapture capture_;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_1;
  int count_;
  int count_2;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}
