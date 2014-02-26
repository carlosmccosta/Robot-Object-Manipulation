#pragma once

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <defines>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define ACTION_POINT_HEAD "/head_traj_controller/point_head_action"
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </defines>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <string>

// ROS includes
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

// Gazebo includes

// project includes

// namespace specific imports to avoid namespace pollution
using std::string;
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// ##################################################   HeadController   ###############################################
/**
 * \brief Description...
 */
class HeadController {
	// =============================================   <public-section>   ==============================================
	public:
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		HeadController();
		virtual ~HeadController();
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <HeadController-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		void lookAt(string frame_id, double x, double y, double z, double max_seconds_wait_for_server = 30.0, double max_seconds_wait_for_result = 30.0);
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </HeadController-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// =============================================   </public-section>   =============================================


	// ============================================   <protected-section>   ============================================
	protected:
		actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> point_head_client_;
	// ============================================   </protected-section>   ===========================================
};
