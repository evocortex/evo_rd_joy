/*
 * JoyToVel.h
 *
 *  Created on: 28.06.2018
 *      Author: chris
 */

#pragma once

// ros includes
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <deque>
#include <unordered_map>

#include "ps4buttonmapping.h"


/**
 * @namespace evo
 *
 */
namespace evo {


/**
 * @namespace Motor
 */
namespace Motor {
constexpr char ENABLE  = 1U;
constexpr char DISABLE = 0U;
}   // namespace Motor


/**
 * @namespace Button
 */
namespace Button {
constexpr bool PRESSED  = true;
constexpr bool RELEASED = false;
}   // namespace Button

/**
 * @namespace Constants
 */
namespace Constants {
constexpr double VEL_INCREMENT   = 0.1;
constexpr double VEL_SLOW_FACTOR = 0.5;
constexpr double TIME_INCREMENT  = 0.1;
}   // namespace Constants

enum class CommandID
{
   enable = 0,
   disable,
   lift_up,
   lift_down,
   toggle_twist,
   dead_man_button,
   dead_man_slow,
   trans_x,
   trans_y,
   rot_z,
   dpad_west,
   dpad_north,
   print_pose
};


/**
 * @struct JoyConfig
 *
 */
struct JoyConfig
{
   double rot_scale = 1.0;
   double lin_scale = 1.0;
   double v_max     = 1.0;
   double r_max     = 1.0;

   unsigned int averaging_window_size = 20;
   u_int8_t lift_v_max  = 127;
};


/**'
 * @class   JoyToVel
 * @author  Christian Pfitzner
 */
class JoyToVel
{
public:
   JoyToVel();

   /**
   * Spin function of ros node
   */
   void run();

private:
   /**
   * Callback function for jostick
   * @param msg
   */
   void joyCallback(const sensor_msgs::Joy& msg);

   /**
   * Function to publish velocities
   */
   void publishVel(const sensor_msgs::Joy& msg);

   /**
   * Function to lift up
   */
   void publishLiftUp() const;
   /**
   * Function to lift down. The speed can be set via the configuration
   */
   void publishLiftDown() const;

   /**
   * Function to do nothing with the lift
   */
   void publishNoLift() const;

   /**
   * Function to enable drives
   */
   void publishEnable();
   /**
   * Function to disable drives
   */
   void publishDisable();

   /**
   * Callback for timer to ensure that the dead man swith is pressed
   * @param e
   */
   void timerCallback(const ros::TimerEvent& e);

   /**
   * Helper function to check whether the button for the given command was pressed
   * @param cmd
   * @param msg
   */
   bool buttonPressed(CommandID cmd, const sensor_msgs::Joy& msg);

   /**
   * Helper function to check whether the button state for a given command changed
   * @param cmd
   * @param msg
   * @param msg_old
   */
   bool buttonChanged(CommandID cmd, const sensor_msgs::Joy& msg, const sensor_msgs::Joy& msg_old);

   /**
   * Helper function to check whether the button state for a given command changed to the given value
   * @param pressed
   * @param cmd
   * @param msg
   * @param msg_old
   */
   bool buttonChangedTo(bool                    pressed,
                        CommandID               cmd,
                        const sensor_msgs::Joy& msg,
                        const sensor_msgs::Joy& msg_old);

   /**
   * Helper function to get axis value for the given command
   * @param msg
   * @param cmd
   */
   double axisValue(const sensor_msgs::Joy& msg, CommandID cmd);

   /**
   * @brief initCmdToButtonMapping - load button mapping from ros params
   * @param private_nh
   */
   void initCmdToButtonMapping(const ros::NodeHandle& private_nh);


   std::deque<double> _avg_lin_x;   //!< averaging deque for linear tranlation in x direction
   std::deque<double> _avg_lin_y;   //!< averaging deque for linear translation in x direction
   std::deque<double> _avg_rot;     //!< averaging deque for rotation


   ros::NodeHandle _nh;   //!< ros node handle

   // subscribers
   ros::Subscriber _joy_sub;   //!< subscriber for joystick

   ros::Publisher _vel_pub;    //!< publisher for velocity messages
   ros::Publisher _lift_pub;   //!< publisher to lift

   ros::ServiceClient _enable_client;   //!< publisher to enable and disable drives


   JoyConfig _joy_config;   //!< configuration for drives

   std::map<CommandID, int> _button_map;
   std::map<CommandID, int> _axis_map;

   bool _is_initialized;   //!< boolean variable if twist is enabled
   bool _twist_disabled;   //!< boolean variable if the publication of twist is disabled

   bool _dead_man_pressed;
   bool _dead_man_slow_pressed;
   bool _published_zero_once;

   tf::StampedTransform     _tf_robot;
   tf::TransformListener    _tf_listener;
   tf::TransformBroadcaster _tf_broadcaster;

   double _limit_vel;
   double _limit_rad;
};


/**
 * Converter Function to limit a variable
 * @param var     variable
 * @param min     minimum value
 * @param max     maximum value
 */
template<typename T>
static void limit_value(T& var, const T min, const T max)
{
   if (min >= max)
   {
      std::cout << "error: minimum must be smaller than maximum" << std::endl;
      return;
   }

   if (var > max)
   {
      var = max;
   }

   if (var < min)
   {
      var = min;
   }
}


} /* namespace evo */