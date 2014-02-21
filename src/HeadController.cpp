/**\file HeadController.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author carloscosta
 */


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include "HeadController.h"
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// ================================================   <public-section>   ===============================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
HeadController::HeadController() : point_head_client_(ACTION_POINT_HEAD, true) {}
HeadController::~HeadController() {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <HeadController-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void HeadController::lookAt(string frame_id, double x, double y, double z, double max_seconds_wait_for_server, double max_seconds_wait_for_result) {
	geometry_msgs::PointStamped point;
	point.header.frame_id = frame_id;
	point.point.x = x;
	point.point.y = y;
	point.point.z = z;

	pr2_controllers_msgs::PointHeadGoal goal;
	goal.target = point;
	goal.pointing_frame = "high_def_frame";
	goal.min_duration = ros::Duration(0.5);
	goal.max_velocity = 1.0;


	ROS_INFO("Waiting for the point_head_action server");
	point_head_client_.waitForServer(ros::Duration(max_seconds_wait_for_server));
	point_head_client_.sendGoal(goal);
	point_head_client_.waitForResult(ros::Duration(max_seconds_wait_for_result));
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </HeadController-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// ================================================   </public-section>   ==============================================
