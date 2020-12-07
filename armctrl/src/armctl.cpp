#include "armctl.h"
#include<ros/ros.h>
#include<ros/console.h>
#include<std_msgs/Float64.h>
#include<string>

namespace arm_controller_ns {

//double *xpos;


//Controller initialization
  bool joint1ControllerClass::init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &nh)
  {
		//Retrieve the joint object to control
		std::string j1joint_name;
		if( !nh.getParam( "j1Joint_name", j1joint_name ) ) {
			ROS_ERROR("No j1joint_name specified");
			return false;
		}
    j1jh = hw->getHandle(j1joint_name);
    return true;
  }

//Controller startup
  void joint1ControllerClass::starting(const ros::Time& time) {

		//Get initial position to use in the control procedure
		j1init_pos = j1jh.getPosition();
		ros::param::set("cmd_yaw", 0);
		ros::param::set("tempyaw", 0.0);
		ros::param::set("yawc", 0.0);
	}

//Controller running
  void joint1ControllerClass::update(const ros::Time& time, const ros::Duration& period)
  {
	  	
		double yaw,tempyaw,yawc;
		ros::param::get("cmd_yaw", yaw);
		ros::param::get("tempyaw", tempyaw);
		ros::param::get("yawc", yawc);
		//300*rov_controller_ns::xpos; // * sin(ros::Time::now().toSec());
		
		//For relative moment
		//********************************
		// if (yaw != tempyaw && yaw != yawc)
		// {
		// ros::param::set("tempyaw", (tempyaw + yaw));
		// ros::param::set("yawc",yaw);
		// ROS_INFO_STREAM(tempyaw);
		//  //Apply command to the selected joint
		// }
		// else
		// {
		// 	j1jh.setCommand( (tempyaw));
		// }
		//********************************************
		j1jh.setCommand( (yaw)); //For Absolute movement
		
  }

//Controller exiting
  void joint1ControllerClass::stopping(const ros::Time& time) { }
/****************************************************************************************************
  bool joint2ControllerClass::init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &nh)
  {
		//Retrieve the joint object to control
		std::string j2joint_name;
		if( !nh.getParam( "j2joint_name", j2joint_name ) ) {
			ROS_ERROR("No j2joint_name specified");
			return false;
		}
    // j1jh = hw->getHandle(joint_name);
	j2jh = hw->getHandle(j2joint_name); 
    return true;
  }

//Controller startup
  void joint2ControllerClass::starting(const ros::Time& time) {

		//Get initial position to use in the control procedure
		j2init_pos = j2jh.getPosition();
	}

//Controller running
  void joint2ControllerClass::update(const ros::Time& time, const ros::Duration& period)
  {
		//---Perform a sinusoidal motion for joint shoulder_pan_joint
		double xpos,zpos;
		ros::param::get("xnnpos", xpos);
		ros::param::get("znnpos", zpos);
		double j2pos = j2init_pos + xpos;//300*rov_controller_ns::xpos;
		//double cpos = joint_.getPosition();
		if(zpos != 0)
		{
			
			j2jh.setCommand(1*zpos*j2pos);

		}
		else{
		j2jh.setCommand( (j2os)); //Apply command to the selected joint
		}
		//---
  }

//Controller exiting
  void RFControllerClass::stopping(const ros::Time& time) { }

    bool LBControllerClass::init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &nh)
  {
		//Retrieve the joint object to control
		std::string lbjoint_name;
		if( !nh.getParam( "lbjoint_name", lbjoint_name ) ) {
			ROS_ERROR("No j2joint_name specified");
			return false;
		}
    // j1jh = hw->getHandle(joint_name);
	lbjh = hw->getHandle(lbjoint_name); 
    return true;
  }

//Controller startup
  void LBControllerClass::starting(const ros::Time& time) {

		//Get initial position to use in the control procedure
		lbinit_pos = lbjh.getPosition();
	}

//Controller running
  void LBControllerClass::update(const ros::Time& time, const ros::Duration& period)
  {
		//---Perform a sinusoidal motion for joint shoulder_pan_joint
		double xpos,zpos;
		ros::param::get("xnnpos", xpos);
		ros::param::get("znnpos", zpos);
		double lbpos = lbinit_pos + xpos;//300*rov_controller_ns::xpos;
		//double cpos = joint_.getPosition();
		if(zpos != 0)
		{
			
			lbjh.setCommand(-1*zpos*lbpos);

		}
		else{
		lbjh.setCommand( (lbpos)); //Apply command to the selected joint
		}
		
		//---
  }

//Controller exiting
  void LBControllerClass::stopping(const ros::Time& time) { }

    bool RBControllerClass::init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &nh)
  {
		//Retrieve the joint object to control
		std::string rbjoint_name;
		if( !nh.getParam( "rbjoint_name", rbjoint_name ) ) {
			ROS_ERROR("No rbjoint_name specified");
			return false;
		}
    // j1jh = hw->getHandle(joint_name);
	rbjh = hw->getHandle(rbjoint_name); 
    return true;
  }

//Controller startup
  void RBControllerClass::starting(const ros::Time& time) {

		//Get initial position to use in the control procedure
		rbinit_pos = rbjh.getPosition();
	}

//Controller running
  void RBControllerClass::update(const ros::Time& time, const ros::Duration& period)
  {
		//---Perform a sinusoidal motion for joint shoulder_pan_joint
		double xpos,zpos;
		ros::param::get("xnnpos", xpos);
		ros::param::get("znnpos", zpos);
		double rbpos = rbinit_pos + xpos;//300*rov_controller_ns::xpos;
		//double cpos = joint_.getPosition();
		if(zpos != 0 )
		{
			
			rbjh.setCommand(1*zpos*rbpos);

		}
		else{
		rbjh.setCommand( (rbpos)); //Apply command to the selected joint
		}
		
		//---
  }

//Controller exiting
  void RBControllerClass::stopping(const ros::Time& time) { }

}
*///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Register the plugin: PLUGINLIB_EXPORT_CLASS(my_namespace::MyPlugin, base_class_namespace::PluginBaseClass)
}
PLUGINLIB_EXPORT_CLASS(arm_controller_ns::joint1ControllerClass, controller_interface::ControllerBase);
//PLUGINLIB_EXPORT_CLASS(rov_controller_ns::joint2ControllerClass, controller_interface::ControllerBase);
//PLUGINLIB_EXPORT_CLASS(rov_controller_ns::LBControllerClass, controller_interface::ControllerBase);
//PLUGINLIB_EXPORT_CLASS(rov_controller_ns::RBControllerClass, controller_interface::ControllerBase);

void teleopcb(const std_msgs::Float64::ConstPtr& msg)
	{
	//if(msg->linear.x != 0)
	//nh.setParam("cmd_yaw", msg->data);
	//nh.setParam("znnpos",msg->angular.z);

	ros::param::set("cmd_yaw", msg->data);
	
	//ros::param::get("znnpos", zinpos);

	
	// *(rov_controller_ns::xpos)=msg->linear.x;
	// ROS_INFO_STREAM(*(rov_controller_ns::xpos));
	//    ROS_INFO_STREAM(rov_controller_ns::xpos);
	
	}
int main(int argc,char** argv)
{	
	
    ros::init(argc,argv,"teleoprecv");
		//double cpos = joint_.getPosition();
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("cmd_yaw",1,teleopcb);
    ros::spin();
}
