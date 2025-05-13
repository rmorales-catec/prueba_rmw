#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
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
  : Node("image_publisher_qos")  //Declaramos el nombre del nodo e inicializamos un contador
  {
    auto image_qos = rclcpp::SystemDefaultsQoS();
    // auto image_qos = rclcpp::SensorDataQoS();
      
    //Definimos el publisher, declaramos el tipo de mensaje, el topic en el que vamos a publicar
    publisher_=this->create_publisher<sensor_msgs::msg::Image>("image", image_qos); 
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
    cv::Mat frame;  //variable donde vamos a guardar la imagen. Es una matriz
    capture_ >> frame; //Guardamos la imagen que hemos capturado en capture_

    if (frame.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Error al capturar la imagen de la cámara.");
      return;
    }

    try {  //intentamos convertir imagen y en caso de que no se pueda publicamos un mensaje de error 
      // Convertir la imagen de OpenCV (cv::Mat) a sensor_msgs::msg::Image
      auto ros_image = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      ros_image->header.stamp = this->get_clock()->now();
      // Publicar la imagen en el tópico "image"
      publisher_->publish(*ros_image);
    } 
    catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error al convertir la imagen: %s", e.what());
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  cv::VideoCapture capture_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}
