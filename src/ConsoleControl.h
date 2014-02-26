#pragma once

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <defines>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define TOPIC_BASE_CONTROLLER "base_controller/command"
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </defines>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// Gazebo includes

// project includes
#include "ObjectManipulation.h"

// namespace specific imports to avoid namespace pollution
using boost::shared_ptr;
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// ##################################################   ConsoleControl   ###############################################
/**
 * \brief Description...
 */
class ConsoleControl {
	// =============================================   <public-section>   ==============================================
	public:
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		ConsoleControl(shared_ptr<ros::NodeHandle> node_handle);
		virtual ~ConsoleControl();
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <ConsoleControl-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		void setupConsole();
		void resetConfiguration();
		bool initObjectManipulation();
		bool resetArms();
		bool liftObject();
		bool placeObject();
		void processInput();
		void showHelp();
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </ConsoleControl-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <static vars>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		static int kfd;
		static termios cooked;
		static termios raw;
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </static vars>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// =============================================   </public-section>   =============================================
		
	// ============================================   <protected-section>   ============================================
	protected:
		  double walk_vel_, run_vel_, yaw_rate_, yaw_rate_run_;
		  geometry_msgs::Twist cmd_;

		  shared_ptr<ros::NodeHandle> node_handle_;
		  ros::Publisher vel_pub_;
		  bool object_manipulation_initialized_;
		  HeadController headController_;
		  ObjectManipulation objectManipulation_;
	// ============================================   </protected-section>   ===========================================
};
