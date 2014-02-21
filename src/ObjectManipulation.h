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
#define ACTION_PICKUP "object_manipulator/object_manipulator_pickup"
#define ACTION_PLACE "object_manipulator/object_manipulator_place"
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </defines>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes

// boost includes
#include <boost/shared_ptr.hpp>

// ROS includes
#include <actionlib/client/simple_action_client.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>

// Gazebo includes

// project includes

// namespace specific imports to avoid namespace pollution
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
		bool setupServices(shared_ptr<NodeHandle> node_handle, double max_seconds_wait_for_service = 30.0);
		bool waitForActionServers(shared_ptr<NodeHandle> node_handle, double max_seconds_wait_for_server = 30.0);
		int detectObjectsOnTable();
		int processCollisions();
		bool pickupObject(float lift_z_distance = 0.1, double max_seconds_wait_for_conclusion = 120.0);
		bool placeObject(double x_offset = -0.1, double y_offset = 0.1, double z_offset = -0.1, double max_seconds_wait_for_conclusion = 120.0);
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </ObjectManipulation-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// =============================================   </public-section>   =============================================

	// ============================================   <protected-section>   ============================================
	protected:
		ServiceClient object_detection_srv_;
		ServiceClient collision_processing_srv_;
		SimpleActionClient<object_manipulation_msgs::PickupAction> pickup_client_;
		SimpleActionClient<object_manipulation_msgs::PlaceAction> place_client_;

		tabletop_object_detector::TabletopDetection detection_call_;
		tabletop_collision_map_processing::TabletopCollisionMapProcessing processing_call_;

		geometry_msgs::Vector3Stamped direction_;
		object_manipulation_msgs::PickupResult pickup_result_;
	// ============================================   </protected-section>   ===========================================
};

