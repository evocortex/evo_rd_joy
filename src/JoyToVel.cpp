/*
 * JoyToVel.cpp
 *
 *  Created on: 28.06.2018
 *      Author: chris
 */

#include "evo_rd_joy/JoyToVel.h"

//ros includes
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

// necessary for accumulate
#include <numeric>

namespace evo {

JoyToVel::JoyToVel()
   : _is_initialized(false)
   , _twist_disabled(false)
   , _dead_man_pressed(false)
   , _dead_man_slow_pressed(false)
   , _published_zero_once(false)
//, _switch_is_released_for_a_certain_time(false)
{
   ros::NodeHandle private_nh("~");

   const std::string joy_topic    = "joy";
   const std::string vel_topic    = "cmd_vel";
   const std::string lift_topic   = "cmd_lift";
   const std::string enable_topic = "drives/enable";

   auto averaging_window_size = 0;
   double v_max                 = 0.0;
   double r_max                 = 0.0;

   // parameters from launch file
   private_nh.param("v_max", v_max, 1.0);
   private_nh.param("r_max", r_max, 1.0);
   private_nh.param("averaging_window_size", averaging_window_size, 1);

   _joy_config.v_max                 = v_max;
   _joy_config.r_max                 = r_max;
   _joy_config.averaging_window_size = averaging_window_size;

   // init subscribers
   _joy_sub = _nh.subscribe(joy_topic, 1, &JoyToVel::joyCallback, this);

   // init publishers
   _vel_pub  = _nh.advertise<geometry_msgs::Twist>(vel_topic, 1);
   _lift_pub = _nh.advertise<std_msgs::Int8>(lift_topic, 1);

   // init services
   _enable_client = _nh.serviceClient<std_srvs::SetBool>("drives/enable");

   initCmdToButtonMapping(private_nh);

   if (_button_map.empty() || _axis_map.empty())
   {
      ROS_ERROR_STREAM("Controller mapping not initialized!");
      exit(1);
   }

   ROS_INFO_STREAM("Finished initialization of evo_rd_joy");

   _avg_lin_x.resize(averaging_window_size);
   _avg_lin_y.resize(averaging_window_size);
   _avg_rot.resize(averaging_window_size);

   _limit_vel = _joy_config.v_max;
   _limit_rad = _joy_config.r_max;
}

void JoyToVel::run()
{
   ros::spin();
}

void JoyToVel::initCmdToButtonMapping(const ros::NodeHandle& private_nh)
{
   _button_map.clear();
   _axis_map.clear();

   std::string prefix = "button_mapping/";

   int value = 0;

   //buttons
   private_nh.param(prefix + "button/enable", value, PS4_BUTTONS::SQUARE);
   _button_map[CommandID::enable] = value;

   private_nh.param(prefix + "button/disable", value, PS4_BUTTONS::CIRCLE);
   _button_map[CommandID::disable] = value;

   private_nh.param(prefix + "button/lift_up", value, PS4_BUTTONS::TRIANGLE);
   _button_map[CommandID::lift_up] = value;

   private_nh.param(prefix + "button/lift_down", value, PS4_BUTTONS::CROSS);
   _button_map[CommandID::lift_down] = value;

   private_nh.param(prefix + "button/toggle_twist", value, PS4_BUTTONS::PS);
   _button_map[CommandID::toggle_twist] = value;

   private_nh.param(prefix + "button/dead_man_button", value, PS4_BUTTONS::R1);
   _button_map[CommandID::dead_man_button] = value;

   private_nh.param(prefix + "button/dead_man_slow", value, PS4_BUTTONS::R2);
   _button_map[CommandID::dead_man_slow] = value;

   private_nh.param(prefix + "button/print_pose", value, PS4_BUTTONS::SHARE);
   _button_map[CommandID::print_pose] = value;

   //axes
   private_nh.param(prefix + "axis/trans_x", value, PS4_AXES::LEFT_NORTH);
   _axis_map[CommandID::trans_x] = value;

   private_nh.param(prefix + "axis/trans_y", value, PS4_AXES::LEFT_WEST);
   _axis_map[CommandID::trans_y] = value;

   private_nh.param(prefix + "axis/rot_z", value, PS4_AXES::RIGHT_WEST);
   _axis_map[CommandID::rot_z] = value;

   private_nh.param(prefix + "axis/west", value, PS4_AXES::DPAD_WEST);
   _axis_map[CommandID::dpad_west] = value;

   private_nh.param(prefix + "axis/north", value, PS4_AXES::DPAD_NORTH);
   _axis_map[CommandID::dpad_north] = value;
}

void JoyToVel::joyCallback(const sensor_msgs::Joy& msg)
{
   // only executed at first call
   static sensor_msgs::Joy msg_old = msg;

   // enable and disable
   if (buttonPressed(CommandID::disable, msg))
   {
      this->publishDisable();
   }
   else if (buttonPressed(CommandID::enable, msg)
            || buttonChangedTo(Button::PRESSED, CommandID::dead_man_button, msg, msg_old)
            || buttonChangedTo(Button::PRESSED, CommandID::dead_man_slow, msg, msg_old))
   {
      this->publishEnable();
   }

   _dead_man_pressed = buttonPressed(CommandID::dead_man_button, msg)
                       || buttonPressed(CommandID::dead_man_slow, msg);

   _dead_man_slow_pressed = buttonPressed(CommandID::dead_man_slow, msg);


   if (buttonChangedTo(Button::PRESSED, CommandID::toggle_twist, msg, msg_old))
   {
      _twist_disabled = !_twist_disabled;
   }

   this->publishVel(msg);

   // up and down of lifting mechanism
   if (!_twist_disabled)
   {
      if (buttonPressed(CommandID::lift_up, msg))
      {
         this->publishLiftUp();
      }
      else if (buttonPressed(CommandID::lift_down, msg))
      {
         this->publishLiftDown();
      }
      else
      {
         this->publishNoLift();
      }
   }

   if (buttonChangedTo(Button::PRESSED, CommandID::print_pose, msg, msg_old))
   {
      ros::Time now = ros::Time::now();
      _tf_listener.waitForTransform("map",
                                    "base_footprint",
                                    now,
                                    ros::Duration(Constants::TIME_INCREMENT));
      try
      {
         _tf_listener.lookupTransform("map", "base_footprint", now, _tf_robot);
         ROS_INFO_STREAM("base_footprint translation (xyz): " << _tf_robot.getOrigin().getX() << " "
                                                              << _tf_robot.getOrigin().getY() << " "
                                                              << _tf_robot.getOrigin().getZ());
         ROS_INFO_STREAM("base_footprint rotation (xyzw): " << _tf_robot.getRotation().getX() << " "
                                                            << _tf_robot.getRotation().getY() << " "
                                                            << _tf_robot.getRotation().getZ() << " "
                                                            << _tf_robot.getRotation().getW());
         ROS_INFO_STREAM("base_footprint posefile line (xy trans, zw rot): "
                         << _tf_robot.getOrigin().getX() << " " << _tf_robot.getOrigin().getY()
                         << " " << _tf_robot.getRotation().getZ() << " "
                         << _tf_robot.getRotation().getW());
      }
      catch (const tf2::TransformException& e)
      {
         ROS_ERROR_STREAM("No valid tf found for frame base_footprint!");
      }
   }

   if (!_dead_man_pressed)
   {
      // button north
      if (axisValue(msg, CommandID::dpad_north) > 0
          && axisValue(msg_old, CommandID::dpad_north) < 1.0)
      {
         _limit_vel += Constants::VEL_INCREMENT;
         _limit_rad += Constants::VEL_INCREMENT;

         limit_value<double>(_limit_vel, Constants::VEL_INCREMENT, _joy_config.v_max);
         limit_value<double>(_limit_rad, Constants::VEL_INCREMENT, _joy_config.r_max);

         ROS_INFO_STREAM("Set new v_max to " << _limit_vel << " and new r_max to " << _limit_rad);
      }
      else if (axisValue(msg, CommandID::dpad_north) < 0
               && axisValue(msg_old, CommandID::dpad_north) > -1.0)
      {
         _limit_vel -= Constants::VEL_INCREMENT;
         _limit_rad -= Constants::VEL_INCREMENT;

         limit_value<double>(_limit_vel, Constants::VEL_INCREMENT, _joy_config.v_max);
         limit_value<double>(_limit_rad, Constants::VEL_INCREMENT, _joy_config.r_max);

         ROS_INFO_STREAM("Set new v_max to " << _limit_vel << " and new r_max to " << _limit_rad);
      }

      // button west
      if (axisValue(msg, CommandID::dpad_west) > 0
          && axisValue(msg_old, CommandID::dpad_west) < 1.0)
      {
         ROS_ERROR_STREAM("button west pressed");
      }
      else if (axisValue(msg, CommandID::dpad_west) < 0
               && axisValue(msg_old, CommandID::dpad_west) > -1.0)
      {
         ROS_ERROR_STREAM("button east pressed");
      }
   }

   // save the current one to the old one for rising trigger
   msg_old = msg;
}

void JoyToVel::publishVel(const sensor_msgs::Joy& msg)
{
   geometry_msgs::Twist twist;

   double v_x = 0.0;
   double v_y = 0.0;
   double r_z = axisValue(msg, CommandID::rot_z);

   if (axisValue(msg, CommandID::dpad_north) != 0)
   {
      v_x = axisValue(msg, CommandID::dpad_north);
   }
   else
   {
      v_x = axisValue(msg, CommandID::trans_x);
   }
   if (axisValue(msg, CommandID::dpad_west) != 0)
   {
      v_y = axisValue(msg, CommandID::dpad_west);
   }
   else
   {
      v_y = axisValue(msg, CommandID::trans_y);
   }

   if (_dead_man_slow_pressed)
   {
      v_x *= Constants::VEL_SLOW_FACTOR;
      v_y *= Constants::VEL_SLOW_FACTOR;
      r_z *= Constants::VEL_SLOW_FACTOR;
   }

   // map Joystick scaling to the current limit
   v_x *= _limit_vel;
   v_y *= _limit_vel;
   r_z *= _limit_rad;

   limit_value<double>(v_x, -_limit_vel, _limit_vel);
   limit_value<double>(v_y, -_limit_vel, _limit_vel);
   limit_value<double>(r_z, -_limit_rad, _limit_rad);

   if (_twist_disabled)
   {
      if (v_x != 0.0 || v_y != 0.0 || r_z != 0.0)
      {
         ROS_WARN_STREAM("Twist is disabled. Please press playstation button for re-activation");
      }
      return;
   }

   // apply square joystick curve
   twist.linear.x  = v_x;   // * std::abs(v_x) * _joy_config.lin_scale;
   twist.linear.y  = v_y;   // * std::abs(v_y) * _joy_config.lin_scale;
   twist.angular.z = r_z;   // * std::abs(r_z) * _joy_config.rot_scale;

   bool no_joystick_moved = true;
   if (twist.linear.x != 0.0)
   {
      no_joystick_moved = false;
   }
   if (twist.linear.y != 0.0)
   {
      no_joystick_moved = false;
   }
   if (twist.angular.x != 0.0)
   {
      no_joystick_moved = false;
   }

   // check if dead man switch is configured and pressed
   if (!_dead_man_pressed)
   {
      ROS_WARN_STREAM_THROTTLE(5, "Dead man switch is released. The robot will stop immediately.");

      // if the switch is released for more than two seconds, do not publish
      // velocity command
      if (!_published_zero_once)
      {
         ROS_DEBUG_STREAM("Publishing zero velocity");
         _vel_pub.publish(geometry_msgs::Twist());
         _published_zero_once = true;
      }
   }
   else
   {
      // _timer_for_deadman.setPeriod(ros::Duration(2.0), true);
      _vel_pub.publish(twist);
      _published_zero_once = false;
   }
}

void JoyToVel::publishLiftUp() const
{
   std_msgs::Int8 up_msg;
   up_msg.data = static_cast<char>(_joy_config.lift_v_max);

   _lift_pub.publish(up_msg);
}

void JoyToVel::publishLiftDown() const
{
   std_msgs::Int8 down_msg;
   down_msg.data = static_cast<char>(-_joy_config.lift_v_max);

   _lift_pub.publish(down_msg);
}

void JoyToVel::publishNoLift() const
{
   std_msgs::Int8 nothing;
   nothing.data = 0;

   _lift_pub.publish(nothing);
}

void JoyToVel::publishEnable()
{
   std_srvs::SetBool srv;
   srv.request.data = Motor::ENABLE;

   if (!_enable_client.call(srv))
   {
      ROS_ERROR_STREAM("Failed to enable drives");
   }
}

void JoyToVel::publishDisable()
{
   std_srvs::SetBool srv;
   srv.request.data = Motor::DISABLE;

   if (!_enable_client.call(srv))
   {
      ROS_ERROR_STREAM("Failed to disable drives");
   }
}

bool JoyToVel::buttonPressed(const CommandID cmd, const sensor_msgs::Joy& msg)
{
   return msg.buttons[_button_map[cmd]] == 1;
}

bool JoyToVel::buttonChanged(const CommandID         cmd,
                             const sensor_msgs::Joy& msg,
                             const sensor_msgs::Joy& msg_old)
{
   return buttonPressed(cmd, msg) != buttonPressed(cmd, msg_old);
}

bool JoyToVel::buttonChangedTo(bool                    pressed,
                               const CommandID         cmd,
                               const sensor_msgs::Joy& msg,
                               const sensor_msgs::Joy& msg_old)
{
   return buttonChanged(cmd, msg, msg_old) && (buttonPressed(cmd, msg) == pressed);
}

double JoyToVel::axisValue(const sensor_msgs::Joy& msg, const CommandID cmd)
{
   return msg.axes[_axis_map[cmd]];
}

/// @todo fix bug for the scaling of the robot's velocity //TODO WHA bug not known, still relevant?
} /* namespace evo */

int main(int argc, char** argv)
{
   ros::init(argc, argv, "evo_rd_joy");

   ROS_INFO_STREAM("Starting evo_rd_joy...");

   evo::JoyToVel node;
   node.run();

   return 0;
}