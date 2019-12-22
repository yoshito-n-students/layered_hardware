#include <controller_manager/controller_manager.h>
#include <layered_hardware/layered_hardware.hpp>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/spinner.h>
#include <ros/time.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "dynamixel_hardware_node");
  ros::NodeHandle nh, pnh("~");

  layered_hardware::LayeredHardware hw;
  if (!hw.init(pnh)) {
    ROS_ERROR("Failed to init LayeredHardware");
    return 1;
  }

  controller_manager::ControllerManager cm(&hw, nh);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate rate(pnh.param("control_frequency", 10.));
  // workaround for clock-jump in some environments
  const bool use_expected_period(pnh.param("use_expected_period", false));
  ros::Time prev_time(ros::Time::now());
  while (ros::ok()) {
    const ros::Time time(ros::Time::now());
    const ros::Duration period(use_expected_period ? rate.expectedCycleTime() : time - prev_time);
    hw.read(time, period);
    cm.update(time, period);
    hw.write(time, period);
    prev_time = time;
    rate.sleep();
  }

  return 0;
}