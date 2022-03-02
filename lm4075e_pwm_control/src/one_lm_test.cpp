#include <lm4075e_pwm_control/one_lm4075e_test.h>

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"Just Test");
    one_actuator test;

    while(ros::ok())
    {
        ros::spinOnce();
        
    }
}