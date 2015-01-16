#ifndef NYMERIA_CONSTANTS_H
#define NYMERIA_CONSTANTS_H

/**
* Declaration of the class NymeriaConstants, that defines all constants necessary
* to define both commands and states of the drone and obstacles.
*/

class NymeriaConstants{
	public:
		NymeriaConstants();
		static const double E_PARAM = -2.0;
		static const int O_FRONT = -1;
		static const int INIT = 0;
		static const int M_FORWARD = 1;
		static const int M_BACKWARD = 2;
		static const int M_LEFT = 3;
		static const int M_RIGHT = 4;
		static const int M_UP = 5;
		static const int M_DOWN = 6;
		static const int T_LEFT = 7;
		static const int T_RIGHT = 8;
		static const int STOP = 9;
		static const int TAKEOFF = 10;
		static const int LAND = 11;
		static const int E_STOP = 12;
		static const int I_M_L_SPEED = 13;
		static const int D_M_L_SPEED = 14;
		static const int I_M_A_SPEED = 15;
		static const int D_M_A_SPEED = 16;
		static const int I_L_SPEED = 17;
		static const int D_L_SPEED = 18;
		static const int I_A_SPEED = 19;
		static const int D_A_SPEED = 20;
		static const int SLOW_DOWN = 21;
		static const double ANTICIPATING_OBSTACLE_DISTANCE = 150.0;
};

#endif
