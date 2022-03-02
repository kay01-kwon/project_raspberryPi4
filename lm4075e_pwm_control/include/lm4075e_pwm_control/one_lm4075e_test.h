#ifndef ONE_LM4075E_TEST_H
#define ONE_LM4075E_TEST_H
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Int32.h>
#include <wiringPi.h>
#include <iostream>
#include <std_msgs/Int32.h>

using std_msgs::Int32;
using std::cout;
using std::endl;

class one_actuator{
    public:
    // Constructor
    one_actuator();

    // Desired Position Callback Function
    void CallbackDesPos(const Int32 & des_pos_msg);
    
    // Encoder Position Callback Function
    void CallbackEncPos(const Int32 & enc_pos_msg);
    
    // PID control
    void PID_control();

    // Destructor
    ~one_actuator();

    private:
    ros::NodeHandle nh;
    ros::Subscriber des_pos_subscriber;
    ros::Subscriber encoder_subscriber;

    double current_time;
    double last_time;
    double dt;
    bool count;

    double Kp, Ki, Kd;

    int Pin_DIR;
    int Pin_PWM;

    int des_pos = 0;
    int enc_pos = 0;

    double pos_error;
    double prev_pos_error;
    double dpos_error;
    double Ipos_error;
    double control_input;

    int max_pwm = 1024;
    int control_input_direction;

};

#endif
