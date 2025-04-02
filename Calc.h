#ifndef __Calc_h__
#define __Calc_h__

typedef struct Arm_Params_t
{
    double L1; // Length of link 1
    double L2; // Length of link 2
    double L3; // Length of link 3

    double theta2; // Joint angle 2
    double theta3; // Joint angle 3

    double q[3]; 

}Arm_Params_t;



#endif // !__Calc_h__
