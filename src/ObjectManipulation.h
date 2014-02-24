#pragma once

/**\file ObjectManipulation.h
 * \brief Description...
 *
 * @version 1.0
 * @author carloscosta
 */



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <defines>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define SERVICE_OBJECT_DETECTION "object_detection"
#define SERVICE_COLLISION_PROCESSING "tabletop_collision_map_processing/tabletop_collision_map_processing"
#define SERVICE_GET_MODEL_DESCRIPTION "objects_database_node/get_model_description"
#define ACTION_PICKUP "object_manipulator/object_manipulator_pickup"
#define ACTION_PLACE "object_manipulator/object_manipulator_place"
#define NAME_LEFT_ARM "left_arm"
#define NAME_RIGHT_ARM "right_arm"
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </defines>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <string>
#include <vector>

// boost includes
#include <boost/shared_ptr.hpp>

// ROS includes
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/collision_map_interface.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>
#include <household_objects_database_msgs/GetModelDescription.h>
#include <object_manipulator/tools/mechanism_interface.h>

// Gazebo includes

// project includes
#include "HeadController.h"


// namespace specific imports to avoid namespace pollution
using std::string;
using std::vector;
using boost::shared_ptr;
using ros::NodeHandle;
using ros::ServiceClient;
using actionlib::SimpleActionClient;
using ros::Duration;
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// ###############################################   ObjectManipulation   ##############################################
/**
 * \brief Description...
 */
class ObjectManipulation {
	// =============================================   <public-section>   ==============================================
	public:
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		ObjectManipulation();
		virtual ~ObjectManipulation();
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <ObjectManipulation-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		bool setupServices(shared_ptr<NodeHandle> node_handle, double max_seconds_wait_for_service = 30.0, bool use_database = true);
		bool waitForActionServers(shared_ptr<NodeHandle> node_handle, double max_seconds_wait_for_server = 30.0);
		int detectObjectsOnTable();
		int processCollisions();
		bool pickupObject(string arm_name = "right_arm", float lift_z_distance = 0.1, double max_seconds_wait_for_conclusion = 120.0);
		bool placeObject(string arm_name = "right_arm", double x_offset = -0.1, double y_offset = 0.1, double z_offset = -0.1, double max_seconds_wait_for_conclusion = 120.0);
		bool moveArmToSide(string arm_name);
		bool tuckArm(string arm_name);
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </ObjectManipulation-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// =============================================   </public-section>   =============================================

	// ============================================   <protected-section>   ============================================
	protected:
		ServiceClient object_detection_srv_;
		ServiceClient collision_processing_srv_;
		ServiceClient get_model_description_srv_;
		SimpleActionClient<object_manipulation_msgs::PickupAction> pickup_client_;
		SimpleActionClient<object_manipulation_msgs::PlaceAction> place_client_;

		tabletop_object_detector::TabletopDetection detection_srv_;
		tabletop_collision_map_processing::CollisionMapInterface collision_map_interface_;
		tabletop_collision_map_processing::TabletopCollisionMapProcessing processing_srv_;
		object_manipulator::MechanismInterface mech_interface_;

		geometry_msgs::Vector3Stamped direction_;
		object_manipulation_msgs::PickupResult pickup_result_;
	// ============================================   </protected-section>   ===========================================
};

