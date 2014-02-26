/**\file ConsoleControl.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author carloscosta
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <defines>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </defines>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include "ConsoleControl.h"
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// ================================================   <public-section>   ===============================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int ConsoleControl::kfd = 0;
termios ConsoleControl::cooked;
termios ConsoleControl::raw;

ConsoleControl::ConsoleControl(shared_ptr<ros::NodeHandle> node_handle) : node_handle_(node_handle), object_manipulation_initialized_(false) {
	setupConsole();
	resetConfiguration();
}

ConsoleControl::~ConsoleControl() {
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <ConsoleControl-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void ConsoleControl::setupConsole() {
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &= ~(ICANON | ECHO);
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);
}

void ConsoleControl::resetConfiguration() {
	cmd_.linear.x = cmd_.linear.y = cmd_.angular.z = 0;

	vel_pub_ = node_handle_->advertise<geometry_msgs::Twist>(TOPIC_BASE_CONTROLLER, 10);

	ros::NodeHandle n_private("~");
	n_private.param("walk_vel", walk_vel_, 0.5);
	n_private.param("run_vel", run_vel_, 1.0);
	n_private.param("yaw_rate", yaw_rate_, 1.0);
	n_private.param("yaw_run_rate", yaw_rate_run_, 1.5);
}


bool ConsoleControl::initObjectManipulation() {
	ROS_INFO("Initializing object manipulation");
	headController_.lookAt("base_link", 0.65, -0.12, 0.55);

	resetArms();

	if (objectManipulation_.setupServices(node_handle_) && objectManipulation_.waitForActionServers(node_handle_)) {
		object_manipulation_initialized_ = true;
		ROS_INFO("Initialization finished");
	} else {
		ROS_ERROR("Initialization failed");
		object_manipulation_initialized_ = false;
	}

	return object_manipulation_initialized_;
}


bool ConsoleControl::resetArms() {
	return (objectManipulation_.moveArmToSide(NAME_LEFT_ARM) && objectManipulation_.moveArmToSide(NAME_RIGHT_ARM));
}


bool ConsoleControl::liftObject() {
	if (!object_manipulation_initialized_) {
		if (!initObjectManipulation()) {
			return false;
		}
	}

	if (objectManipulation_.detectObjectsOnTable() > 0 && objectManipulation_.processCollisions() > 0 && objectManipulation_.pickupObject()) {
		return true;
	}

	return false;
}


bool ConsoleControl::placeObject() {
	if (object_manipulation_initialized_&& objectManipulation_.placeObject()) {
		objectManipulation_.moveArmToSide(NAME_LEFT_ARM);
		objectManipulation_.moveArmToSide(NAME_RIGHT_ARM);
		return true;
	}

	return false;
}


void ConsoleControl::processInput() {
	showHelp();

	char c;
	bool movementInput;
	while (node_handle_->ok()) {
		if (read(kfd, &c, 1) < 0) {
			perror("read() failed:");
			exit(-1);
		}

		cmd_.linear.x = cmd_.linear.y = cmd_.angular.z = 0;

		movementInput = true;
		switch (c) {
			case 'w': 	cmd_.linear.x  =  walk_vel_; 			break;
			case 's': 	cmd_.linear.x  = -walk_vel_; 			break;
			case 'a': 	cmd_.linear.y  =  walk_vel_; 			break;
			case 'd': 	cmd_.linear.y  = -walk_vel_; 			break;
			case 'q': 	cmd_.angular.z =  yaw_rate_; 			break;
			case 'e': 	cmd_.angular.z = -yaw_rate_; 			break;
			case 'W':	cmd_.linear.x  =  run_vel_; 			break;
			case 'S':	cmd_.linear.x  = -run_vel_; 			break;
			case 'A':	cmd_.linear.y  =  run_vel_; 			break;
			case 'D':	cmd_.linear.y  = -run_vel_; 			break;
			case 'Q':	cmd_.angular.z =  yaw_rate_run_; 		break;
			case 'E':	cmd_.angular.z = -yaw_rate_run_; 		break;
			case 'l':	liftObject();	movementInput = false;  break;
			case 'p':	placeObject();  movementInput = false;  break;
			case 'r':	resetArms();  	movementInput = false;  break;
			case 'h':	showHelp();  	movementInput = false;  break;
			default : 	movementInput  = false; 				break;
		}

		if (movementInput) {
			vel_pub_.publish(cmd_);
		}
	}
}


void ConsoleControl::showHelp() {
	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use 'wasd' to translate");
	puts("Use 'qe' to yaw");
	puts("Use 'l' to lift objects");
	puts("Use 'p' to place objects");
	puts("Use 'r' to reset arms");
	puts("Press 'Shift' or 'CAPSLOCK' to run");
	puts("Press 'h' to show help");
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </ConsoleControl-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =================================================  <public-section>   ===============================================



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <main>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void quit(int sig) {
	tcsetattr(ConsoleControl::kfd, TCSANOW, &ConsoleControl::cooked);
	exit(0);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "console_control");
	shared_ptr<ros::NodeHandle> node_handle(new ros::NodeHandle());

	signal(SIGINT, quit);

	ConsoleControl consoleControl(node_handle);
	consoleControl.processInput();

	return 0;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </main>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
