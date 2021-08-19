#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "vl53l1x.h"  // Library for the range sensor

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Vl53l1xPublisher : public rclcpp::Node
{
  public:
    Vl53l1xPublisher()
    : Node("VL53L1X_publisher"), count_(0)
    {
      // Set a 500ms timeout on the sensor. (Stop waiting and respond with an error)
      sensor.setTimeout(500);

      if (!sensor.init()) {
        RCLCPP_ERROR(this->get_logger(), "Sensor offline!");
      }

      // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
      // You can change these settings to adjust the performance of the sensor, but
      // the minimum timing budget is 20 ms for short distance mode and 33 ms for
      // medium and long distance modes. See the VL53L1X datasheet for more
      // information on range and timing limits.
      sensor.setDistanceMode(Vl53l1x::Long);
      sensor.setMeasurementTimingBudget(50000);

      // Start continuous readings at a rate of one measurement every 50 ms (the
      // inter-measurement period). This period should be at least as long as the
      // timing budget.
      sensor.startContinuous(100);

      // Setup the publisher
      publisher_ = this->create_publisher<sensor_msgs::msg::Range>("/vl53l1x/range", 5);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&Vl53l1xPublisher::timer_callback, this));
    }

  private:
    Vl53l1x sensor;

    void timer_callback()
    {
      int distance = sensor.read_range();
      if(sensor.timeoutOccurred()) {
        RCLCPP_ERROR(this->get_logger(), "Timeout Occured!");
        distance = 0;
      }

      auto message = sensor_msgs::msg::Range();
      message.radiation_type = sensor_msgs::msg::Range::INFRARED;
      message.field_of_view = 0.47;              // Typically 27 degrees or 0,471239 radians
      message.min_range = 0.14;                  // 140 mm.  (It is actully much less, but this makes sense in the context
      message.max_range = 3.00;                  // 3.6 m. in the dark, down to 73cm in bright light

      message.range = (float) distance / 1000.0; // range in meters

      // from https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Range.msg
      // # (Note: values < range_min or > range_max should be discarded)
      if((message.range >= message.min_range) && (message.range <= message.max_range)) {
        publisher_->publish(message);
      }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vl53l1xPublisher>());
  rclcpp::shutdown();
  return 0;
}