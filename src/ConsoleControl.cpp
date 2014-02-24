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

ConsoleControl::ConsoleControl(shared_ptr<ros::NodeHandle> node_handle) : node_handle_(node_handle) {
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

void ConsoleControl::processInput() {
	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use 'WASD' to translate");
	puts("Use 'QE' to yaw");
	puts("Press 'Shift' to run");

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
			case 'w': 	cmd_.linear.x  =  walk_vel_; 		break;
			case 's': 	cmd_.linear.x  = -walk_vel_; 		break;
			case 'a': 	cmd_.linear.y  =  walk_vel_; 		break;
			case 'd': 	cmd_.linear.y  = -walk_vel_; 		break;
			case 'q': 	cmd_.angular.z =  yaw_rate_; 		break;
			case 'e': 	cmd_.angular.z = -yaw_rate_; 		break;
			case 'W':	cmd_.linear.x  =  run_vel_; 		break;
			case 'S':	cmd_.linear.x  = -run_vel_; 		break;
			case 'A':	cmd_.linear.y  =  run_vel_; 		break;
			case 'D':	cmd_.linear.y  = -run_vel_; 		break;
			case 'Q':	cmd_.angular.z =  yaw_rate_run_; 	break;
			case 'E':	cmd_.angular.z = -yaw_rate_run_; 	break;
			default : 	movementInput = false; 			break;
		}

		if (movementInput) {
			vel_pub_.publish(cmd_);
		}
	}
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </ConsoleControl-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <gets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </gets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <sets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </sets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =================================================  <public-section>   ===============================================

// ===============================================   <protected-section>   =============================================

// ===============================================   </protected-section>   ============================================

// =================================================   <private-section>   =============================================

// =================================================   <private-section>   =============================================


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
