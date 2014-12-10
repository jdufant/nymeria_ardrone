#include <nymeria_ardrone/TeleopKeyboard.h>

void callback (const ardrone_autonomy::Navdata& data){}

ros::NodeHandle * TeleopKeyboard::getNH(){
	return &nh;
}

TeleopKeyboard::TeleopKeyboard(){}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_keyboard");
	TeleopKeyboard teleop;
	signal(SIGINT,quit);

	teleop.keyLoop(teleop.getNH());

	return(0);
}


void TeleopKeyboard::keyLoop(ros::NodeHandle * n)
{
	char c;
	bool dirty(false), flying(false), hovering(true);
	std_msgs::Empty empty_msg;

	Nymeria nymeria(n, 100);

	int rate = 10;
	ros::Rate loop_rate(rate);

	// get the console in raw mode                                                              
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	// Setting a new line, then end of file                        
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("Reading from the keyboard");
	puts("---------------------------");

	puts("spacebar : hovering mode");
	puts("");
	puts("z/s : move forward/backward");
	puts("q/d : rotate left/right");
	puts("<up key>/<down key> : move up/down");
	puts("");

	puts("enter : takeoff/landing");
	puts("");

	/*puts("y/h : increase/decrease max speeds by 10");
	puts("u/j : increase/decrease only linear speed by 10%");
	puts("i/k : increase/decrease only angular speed by 10 percent");
	puts("");*/

	puts("anything else : stop");
	puts("CTRL-C to quit");
	puts("---------------------------");

	int count=0;

	while(ros::ok()){
		count++;
		// get the next event from the keyboard  
		if(read(kfd, &c, 1) < 0){
			perror("read():");
			exit(-1);
		}

		ROS_INFO("value: 0x%02X\n", c);
		//std::stringstream ss;

		switch(c){
			case KEYCODE_q:
				nymeria.turnLeft();
				break;	

			case KEYCODE_d:
				nymeria.turnRight();
				break;

			case KEYCODE_z:
				nymeria.moveForward();
				break;

			case KEYCODE_s:
				nymeria.moveBackward();
				break;

			case KEYCODE_UP:
				nymeria.moveUp();
				break;

			case KEYCODE_DOWN:
				nymeria.moveDown();
				break;

			case KEYCODE_SPACEBAR:
				nymeria.stop();
				break;

			case KEYCODE_ENTER:
				if (!flying) {
					nymeria.takeOff();
					flying = true;
				}
				else {
					nymeria.land();
					flying = false;
				}
				break;				

			case KEYCODE_a:
				if(hovering == false){
					ROS_WARN("Hovering mode ON");
					hovering = true;
				}

				else {
					ROS_WARN("Hovering mode OFF");
					hovering = false;
				}
				break;

			}

		loop_rate.sleep();
	}
	return;
}
