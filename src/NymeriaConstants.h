/**
 * Declaration of the class NymeriaConstants, that defines all constants necessary
 * to define both commands and states of the drone and obstacles.
 */

class NymeriaConstants{
  public:
    NymeriaConstants();
    static const int CHECK = 0;
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
    static const int I_M_SPEED = 13;
    static const int D_M_SPEED = 14;
    static const int I_L_SPEED = 15;
    static const int D_L_SPEED = 16;
    static const int I_A_SPEED = 17;
    static const int D_A_SPEED = 18;
    static const int NO_OBSTACLE = 20;
    static const int O_FRONT = 21;
};
