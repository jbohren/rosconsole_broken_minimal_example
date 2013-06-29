
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <gazebo_ros_control/robot_hw_sim.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace irk_gazebo_plugins {

  class RobotSimMTM : public gazebo_ros_control::RobotHWSim
  {
  public:
    RobotSimMTM() :
    {   
    }   


    bool initSim(ros::NodeHandle nh, gazebo::physics::ModelPtr model)
    {

      return true;
    }

    void readSim(ros::Time time, ros::Duration period)
    {
    }

    void writeSim(ros::Time time, ros::Duration period)
    {
    }

  private:
  };

}

PLUGINLIB_DECLARE_CLASS(
    irk_gazebo_plugins,
    RobotSimMTM,
    irk_gazebo_plugins::RobotSimMTM, 
    gazebo_ros_control::RobotHWSim)
