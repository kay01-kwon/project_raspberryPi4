#include <lm4075e_pwm_control/lm4075e_control.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"PWM_Control");

    lm4075e_control low_level_controller;

    while(ros::ok())
    {
        ros::spinOnce();
    }

}