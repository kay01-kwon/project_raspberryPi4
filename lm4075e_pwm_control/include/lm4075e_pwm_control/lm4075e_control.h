#ifndef LM4075E_CONTROL_H
#define LM4075E_CONTROL_H
#include <ros/ros.h>
#include <iostream>
#include <lm4075e_pwm_control/lm4075e_pwm_definition.h>

using lm4075e_msgs::Int32;

class lm4075e_control{
    
    public:
    // Constructor
    lm4075e_control();

    // Desired Position Callback Function
    void CallbackDesPos(const Int32 & des_pos_msg);
    
    // Encoder Position Callback Function
    void CallbackEncPos(const Int32 & enc_pos_msg);

    // PID control
    void PID_control();

    // Destructor
    ~lm4075e_control();

    private:
    ros::NodeHandle nh;
    ros::Subscriber des_pos_subscriber;
    ros::Subscriber encoder_subscriber;
    
    double current_time;
    double last_time;
    double dt;
    bool count;

};


// Constructor : Initiate
lm4075e_control::lm4075e_control()
{
    std::cout<<"Constructor"<<std::endl;
    // Pin Mode Setup to do PWM
    wiringPiSetup();
    
    // Direction Pin = 0 : Push, Direction Pin = 1 : Pull
    pinMode(Left_Pin_DIR,OUTPUT);
    pinMode(Left_Pin_DIR,PWM_OUTPUT);

    pinMode(Right_Pin_DIR,OUTPUT);
    pinMode(Right_Pin_PWM,PWM_OUTPUT);

    // Subscriber Setup
    des_pos_subscriber = nh.subscribe("des_pos",1,&lm4075e_control::CallbackDesPos,this);
    encoder_subscriber = nh.subscribe("encoder_data",1,&lm4075e_control::CallbackEncPos,this);

    // Get Gain Parameter
    Kp = 100;
    Kd = 0.1;
    //nh.getParam("Kp",Kp);
    //nh.getParam("Kd",Kd);
    //nh.getParam("Ki",Ki);

    // ROS time
    current_time = ros::Time::now().toSec();
    last_time = current_time;
    dt = 0.01;

    count = false;
}

// Desired Position Callback Function
void lm4075e_control::CallbackDesPos(const Int32 & des_pos_msg)
{
    des_pos_l = des_pos_msg.left_pos;
    des_pos_r = des_pos_msg.right_pos;

}

// Encoder Position Callback Function
void lm4075e_control::CallbackEncPos(const Int32 & enc_pos_msg)
{
    current_time = ros::Time::now().toSec();

    if(count){
        dt = current_time - last_time;
    }else
        count = true;

    enc_pos_l = enc_pos_msg.left_pos;
    enc_pos_r = enc_pos_msg.right_pos;

    PID_control();



    last_time = current_time;
}

// PID control
void lm4075e_control::PID_control()
{
    pos_error_l =  int (des_pos_l - enc_pos_l);
    pos_error_r =  int (des_pos_r - enc_pos_r);

    if(isnan(dt) == true)
        dt = 0.01;

    dpos_error_l = (pos_error_l - pos_prev_error_l)/dt;
    dpos_error_r = (pos_error_r - pos_prev_error_r)/dt;

    control_input_l = (int) Kp*pos_error_l + Kd*dpos_error_l;
    control_input_r = (int) Kp*pos_error_r + Kd*dpos_error_r;
    
    //std::cout<<"Des Pos: "<<des_pos_l;
    //std::cout<<"  Enc Pos: "<<enc_pos_l<<std::endl;
    std::cout<<"control left: "<<control_input_l<<std::endl;
    std::cout<<"control right: "<<control_input_r<<std::endl;
    if(control_input_l > 0)
        control_dir_l = 0;
    else
        control_dir_l = 1;

    if(control_input_r > 0)
        control_dir_r = 0;
    else
        control_dir_r = 1;

    if(abs(control_input_l)  > max_pwm)
        control_input_l = max_pwm;
    
    if(abs(control_input_r)  > max_pwm)
        control_input_r = max_pwm;
    
    digitalWrite(Left_Pin_DIR,control_dir_l);
    pwmWrite(Left_Pin_PWM,abs(control_input_l));

    digitalWrite(Right_Pin_DIR,control_dir_r);
    pwmWrite(Right_Pin_PWM,abs(control_input_r));
    
    pos_prev_error_l = pos_error_l;
    pos_prev_error_r = pos_error_r;
    

}

// Destructor : Delete!
lm4075e_control::~lm4075e_control()
{
//    nh.~NodeHandle();
//    des_pos_subscriber.~Subscriber();
//    encoder_subscriber.~Subscriber();
}

#endif
