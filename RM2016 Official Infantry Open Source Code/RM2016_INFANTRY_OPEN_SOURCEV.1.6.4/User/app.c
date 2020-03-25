#include "main.h"

int Fuyang_Number = 0;
int Xuanzhuan_Number = 0;

int Fuyang_Angle = 0;
int Xuanzhuan_Angle = 0;

void Set_Fuyang(int Give_PWM,int Give_PWM_Location)
{
    Motor_PWM_Location_Set(MOTOR_NUM3,Give_PWM,Give_PWM_Location);
    Fuyang_Number = Give_PWM_Location;
}

void Set_Xuanzhuan(int Give_PWM,int Give_PWM_Location)
{
    Motor_PWM_Location_Set(MOTOR_NUM2,Give_PWM,Give_PWM_Location);
    Xuanzhuan_Number = Give_PWM_Location;
}
