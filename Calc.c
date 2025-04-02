#include <stdio.h>
#include <math.h>
#include "Calc.h"

#define PI 3.14159265

Arm_Params_t Arm_params;

double clamp(double value, double min, double max) {
    return (value < min) ? min : (value > max) ? max : value;
}

int main(){
    double target,X_B,Y_B;
    printf("type in  target X_B Y_B: ");
    scanf("%lf %lf %lf", &target, &X_B, &Y_B);
    Arm_params.B[0] = X_B;
    Arm_params.B[1] = Y_B;
    Arm_params.target = target;
    Arm_params.L1 = 160.0;
    Arm_params.L2 = sqrt(73*73+164*164);
    Arm_params.L3 = 45.0;
    Arm_params.theta2 = atan(73.0/164.0);
    Arm_params.theta3 = 1.6580638556;


    // 计算中间变量
    const double R = sqrt(Arm_params.B[0]*Arm_params.B[0] + Arm_params.B[1]*Arm_params.B[1]);

    // --------------- 检查分母和参数有效性 ---------------
    if (R == 0 || 2*Arm_params.L1*R == 0 || 2*Arm_params.L1*Arm_params.L2 == 0) {
        printf("错误：输入参数导致除零错误\n");
        return 1;
    }

    // 计算 q1 -------------------------------------------------
    double arg_q1 = (Arm_params.L2*Arm_params.L2 - R*R - Arm_params.L1*Arm_params.L1) / (2 * Arm_params.L1 * R);
    if (fabs(arg_q1) > 1.0) {
        printf("错误：q1无解，参数超出范围\n");
        return 1;
    }
    Arm_params.q[0] = -atan2(Y_B, X_B) + acos(arg_q1);
    // q1 = fmod(q1 + 2*PI, 2*PI);    // 归一化到 [0, 2π)
    // if (q1 > PI) q1 -= 2*PI;       // 映射到 [-π, π]
    Arm_params.q[0] = clamp(Arm_params.q[0], 0.0, PI);       // 限制在 [0, π]

    // 计算 q2 -------------------------------------------------
    double arg_q2 = (Arm_params.L1 * Arm_params.L1 + Arm_params.L2 * Arm_params.L2 - R*R) / (2 * Arm_params.L1 * Arm_params.L2);
    if (fabs(arg_q2) > 1.0) {
        printf("错误：q2无解，参数超出范围\n");
        return 2;
    }
    Arm_params.q[1] = acos(arg_q2) - Arm_params.theta2;
    Arm_params.q[1] = clamp(Arm_params.q[1], 0.0, PI);       // 限制在 [0, π]

    // 计算 q3 -------------------------------------------------
    Arm_params.q[3] = target - Arm_params.q[1] - Arm_params.theta2 + Arm_params.q[0] + Arm_params.theta3;
    Arm_params.q[3] = clamp(Arm_params.q[3], -1.66, 1.66);   // 限制在 [-1.66, 1.66]

    // //运行正运动学求解进行比较判断逆解正确性
    // Arm_params.B_resume[0] = Arm_params.L1*cos(PI-Arm_params.q[0])+Arm_params.L2*cos(Arm_params.q[2]+Arm_params.theta2-Arm_params.q[1]);
    // Arm_params.B_resume[1] = Arm_params.L1*sin(PI-Arm_params.q[0])+Arm_params.L2*sin(Arm_params.q[2]+Arm_params.theta2-Arm_params.q[1]);
    // // if(fabs(Arm_params.B_resume[0]-Arm_params.B[0]) > 0.01 || fabs(Arm_params.B_resume[1]-Arm_params.B[1]) > 0.01)
    // // {
    // //     printf("错误：逆解误差过大");
    // //     return 3;
    // // }

    // 输出结果
    printf("q1 = %.4f rad (%.2f°)\n", Arm_params.q[0], Arm_params.q[0]*180/PI);
    printf("q2 = %.4f rad (%.2f°)\n", Arm_params.q[1], Arm_params.q[1]*180/PI);
    printf("q3 = %.4f rad (%.2f°)\n", Arm_params.q[3], Arm_params.q[3]*180/PI);

    return 0;

}
