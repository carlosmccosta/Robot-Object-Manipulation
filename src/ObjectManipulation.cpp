/**\file ObjectManipulation.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author carloscosta
 */



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include "ObjectManipulation.h"
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// ================================================   <public-section>   ===============================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
ObjectManipulation::ObjectManipulation() : pickup_client_(ACTION_PICKUP, true), place_client_(ACTION_PLACE, true) {
	detection_srv_.request.return_clusters = true;
	detection_srv_.request.return_models = true;
	detection_srv_.request.num_models = 1;

	processing_srv_.request.reset_collision_models = true;
	processing_srv_.request.reset_attached_models = true;
	processing_srv_.request.desired_frame = "base_link";

	collision_map_interface_.connectionsEstablished(ros::Duration(-1.0));
}

ObjectManipulation::~ObjectManipulation() {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <ObjectManipulation-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
bool ObjectManipulation::setupServices(shared_ptr<NodeHandle> node_handle, double max_seconds_wait_for_service, bool use_database) {
	ROS_INFO("Waiting for object detection service");
	if (!ros::service::waitForService(SERVICE_OBJECT_DETECTION, Duration(max_seconds_wait_for_service)) || !node_handle->ok()) {
		return false;
	}
	object_detection_srv_ = node_handle->serviceClient<tabletop_object_detector::TabletopDetection>(SERVICE_OBJECT_DETECTION, true);


	ROS_INFO("Waiting for collision processing service");
	if (!ros::service::waitForService(SERVICE_COLLISION_PROCESSING, Duration(max_seconds_wait_for_service)) || !node_handle->ok()) {
		return false;
	}
	collision_processing_srv_ = node_handle->serviceClient<tabletop_collision_map_processing::TabletopCollisionMapProcessing>(SERVICE_COLLISION_PROCESSING, true);


	if (use_database) {
		ROS_INFO("Waiting for %s service to come up", SERVICE_GET_MODEL_DESCRIPTION);
		if (!ros::service::waitForService(SERVICE_GET_MODEL_DESCRIPTION, Duration(max_seconds_wait_for_service)) || !node_handle->ok()) {
			return false;
		}
		get_model_description_srv_ = node_handle->serviceClient<household_objects_database_msgs::GetModelDescription>(string(SERVICE_GET_MODEL_DESCRIPTION), true);
	}

	return true;
}


bool ObjectManipulation::waitForActionServers(shared_ptr<NodeHandle> node_handle, double max_seconds_wait_for_server) {
	ROS_INFO_STREAM("Waiting for pickup action client " << ACTION_PICKUP);
	if (!pickup_client_.waitForServer(Duration(max_seconds_wait_for_server)) || !node_handle->ok()) {
		return false;
	}

	ROS_INFO_STREAM("Waiting for place action client " << ACTION_PLACE);
	if (!place_client_.waitForServer(Duration(max_seconds_wait_for_server)) || !node_handle->ok()) {
		return false;
	}

	return true;
}


int ObjectManipulation::detectObjectsOnTable() {
	ROS_INFO("Calling tabletop detector");

	if (!object_detection_srv_.call(detection_srv_)) {
		ROS_ERROR("Tabletop detection service failed");
		return -1;
	}

	if (detection_srv_.response.detection.result != detection_srv_.response.detection.SUCCESS) {
		ROS_ERROR("Tabletop detection returned error code %d", detection_srv_.response.detection.result);
		return -2;
	}

	if (detection_srv_.response.detection.clusters.empty() && detection_srv_.response.detection.models.empty()) {
		ROS_ERROR("The tabletop detector detected the table, but found no objects");
		return 0;
	}

	return detection_srv_.response.detection.models.size();
}


int ObjectManipulation::processCollisions() {
	ROS_INFO("Calling collision map processing");
	processing_srv_.request.detection_result = detection_srv_.response.detection;

	if (!collision_processing_srv_.call(processing_srv_)) {
		ROS_ERROR("Collision map processing service failed");
		return -1;
	}

	if (processing_srv_.response.graspable_objects.empty()) {
		ROS_ERROR("Collision map processing returned no graspable objects");
	}

	return processing_srv_.response.graspable_objects.size();
}


bool ObjectManipulation::pickupObject(string arm_name, float lift_z_distance, double max_seconds_wait_for_conclusion) {
	if (processing_srv_.response.graspable_objects.empty() || processing_srv_.response.collision_object_names.empty()) {
		return false;
	}

	direction_.header.stamp = ros::Time::now();
	direction_.header.frame_id = "base_link";
	direction_.vector.x = 0;
	direction_.vector.y = 0;
	direction_.vector.z = 1;

	object_manipulation_msgs::PickupGoal pickup_goal;
	pickup_goal.target = processing_srv_.response.graspable_objects.at(0);
	pickup_goal.collision_object_name = processing_srv_.response.collision_object_names.at(0);
	pickup_goal.collision_support_surface_name = processing_srv_.response.collision_support_surface_name;
	pickup_goal.arm_name = arm_name;
	pickup_goal.use_reactive_lift = false;
	pickup_goal.use_reactive_execution = false;
	pickup_goal.lift.direction = direction_;
	pickup_goal.lift.desired_distance = lift_z_distance; //cm
	pickup_goal.lift.min_distance = 0.025;
	pickup_goal.ignore_collisions = 1;

	ROS_INFO("Calling the pickup action");
	pickup_client_.sendGoal(pickup_goal);

	ROS_INFO("Waiting for the pickup action...");
	if (!pickup_client_.waitForResult(Duration(max_seconds_wait_for_conclusion))) {
		pickup_result_ = *(pickup_client_.getResult());
		ROS_ERROR("The pickup action reached the time limit");
		return false;
	}

	pickup_result_ = *(pickup_client_.getResult());
	if (pickup_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_ERROR("The pickup action has failed with result code %d", pickup_result_.manipulation_result.value);
//		mech_interface_.translateGripperCartesian(arm_name, direction_, ros::Duration(2.0), .015, .09, .02, .16, .1);
		return false;
	}

	return true;
}


bool ObjectManipulation::placeObject(string arm_name, double x_offset, double y_offset, double z_offset, double max_seconds_wait_for_conclusion) {
	if (processing_srv_.response.graspable_objects.empty() || processing_srv_.response.collision_object_names.empty()) {
		return false;
	}

	geometry_msgs::PoseStamped place_location;
	place_location.header.frame_id = processing_srv_.response.graspable_objects.at(0).reference_frame_id;
	place_location.pose.orientation.w = 1;
	place_location.header.stamp = ros::Time::now();
	place_location.pose.position.x += x_offset;
	place_location.pose.position.y += y_offset;

	direction_.header.stamp = ros::Time::now();
	direction_.header.frame_id = "base_link";
	direction_.vector.x = 0;
	direction_.vector.y = 0;
	direction_.vector.z = -1;

	ROS_INFO("Calling the place action");
	object_manipulation_msgs::PlaceGoal place_goal;
	place_goal.place_locations.push_back(place_location);
	place_goal.collision_object_name = processing_srv_.response.collision_object_names.at(0);
	place_goal.collision_support_surface_name = processing_srv_.response.collision_support_surface_name;
	place_goal.grasp = pickup_result_.grasp;
	place_goal.arm_name = arm_name;
	place_goal.place_padding = 0.02;
	place_goal.desired_retreat_distance = 0.1;
	place_goal.min_retreat_distance = 0.05;
	place_goal.approach.direction = direction_;
	place_goal.approach.desired_distance = z_offset;
	place_goal.approach.min_distance = 0.05;
	place_goal.use_reactive_place = false;


	place_client_.sendGoal(place_goal);
	ROS_INFO("Waiting for the place action...");
	if (!place_client_.waitForResult(Duration(max_seconds_wait_for_conclusion))) {
		return false;
	}

	object_manipulation_msgs::PlaceResult place_result = *(place_client_.getResult());
	if (place_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_ERROR("Place failed with error code %d", place_result.manipulation_result.value);
		return false;
	}

	ROS_INFO("Object moved");
	return true;
}


bool ObjectManipulation::moveArmToSide(string arm_name) {
	vector<double> side_position(7, 0.0);
	if (arm_name == "right_arm") {
		side_position[0] = -1.135;
		side_position[1] = 0.103;
		side_position[2] = -1.032;
		side_position[3] = -1.005;
		side_position[4] = -1.369;
		side_position[5] = -1.080;
		side_position[6] = 1.398;
	} else {
		side_position[0] = 1.835;
		side_position[1] = 0.103;
		side_position[2] = 0.732;
		side_position[3] = -1.905;
		side_position[4] = 1.369;
		side_position[5] = -0.680;
		side_position[6] = 1.398;
	}

	try {
		arm_navigation_msgs::OrderedCollisionOperations empty_col;
		vector<arm_navigation_msgs::LinkPadding> empty_pad;
		if (!mech_interface_.attemptMoveArmToGoal(arm_name, side_position, empty_col, empty_pad)) {
			ROS_ERROR("Failed to move arm to side\n");
			return false;
		}
	} catch (object_manipulator::MoveArmStuckException &ex) {
		ROS_ERROR("Arm is stuck!\n");
		return false;
	}

	ROS_INFO("Moved arm to side\n");
	return true;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </ObjectManipulation-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =================================================  <public-section>   ===============================================

