#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <wiringPi.h> //Wiring Pi dependency

#include "VL53L1X.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

static VL53L1X distanceSensor;

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Vl53l1xPublisher : public rclcpp::Node
{
  public:
    Vl53l1xPublisher()
    : Node("VL53L1X_publisher"), count_(0)
    {

      // Setup the VL53L1X distance sensor
      Wire.begin();
      
      // delay(200);
      Serial.begin(115200);

      Serial.println("VL53L1X Qwiic Test"); //Why this line??

      if (distanceSensor.begin() == false) {
        RCLCPP_ERROR(this->get_logger(), "Sensor offline!");
      }
 
      // Setup the publisher
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&Vl53l1xPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      while (distanceSensor.newDataReady() == false)
        delay(5);  //Correct this to a ROS2 compliant delay
      int distance = distanceSensor.getDistance(); // Get the result of the measurement from the sensor

      auto message = std_msgs::msg::String();
      message.data = "distance " + std::to_string(distance);
      RCLCPP_INFO(this->get_logger(), "VL53L1X '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vl53l1xPublisher>());
  rclcpp::shutdown();
  return 0;
}
