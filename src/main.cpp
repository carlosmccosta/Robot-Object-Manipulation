/**\file main.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author carloscosta
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <defines>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define NODE_NAME "robot_object_manipulation"
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </defines>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// boost includes
#include <boost/shared_ptr.hpp>

// project includes
#include "HeadController.h"
#include "ObjectManipulation.h"

// namespace specific imports to avoid namespace pollution
using boost::shared_ptr;
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <main>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv) {
	ros::init(argc, argv, NODE_NAME);
	shared_ptr<ros::NodeHandle> node_handle(new ros::NodeHandle());

	sleep(20);

	HeadController headController;
	headController.lookAt("base_link", 1.0, 0.0, 0.5);

	ObjectManipulation objectManipulation;
	if (objectManipulation.setupServices(node_handle) && objectManipulation.waitForActionServers(node_handle)) {
		for (size_t i = 0; i < 3; ++i) {
			if (objectManipulation.detectObjectsOnTable() > 0 && objectManipulation.processCollisions() > 0) {
				if (objectManipulation.pickupObject()) {
					if (objectManipulation.placeObject())
						return 0;
				}
			}
			sleep(10);
		}
	}

	return -1;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </main>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
