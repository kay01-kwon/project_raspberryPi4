#include "one_lm4075e_test.h"

one_actuator::one_actuator()
{
    cout<<"Constructor"<<endl;
    cout<<"Wiring Pi Setup"<<endl;
    wiringPiSetup();

    cout<<"*** Pin PWM and Direction Setup Start ***"<<endl;

    nh.getParam("Pin_PWM",Pin_PWM);
    nh.getParam("Pin_DIR",Pin_DIR);

    cout<<"Pin PWM Setup: "<<Pin_PWM<<endl;
    cout<<"Pin Dir Setup: "<<Pin_DIR<<endl;
    
    pinMode(Pin_DIR,OUTPUT);
    pinMode(Pin_PWM,PWM_OUTPUT);

    cout<<"*** Gain Parameter Setup Start ***"<<endl;

    nh.getParam("Kp",Kp);
    nh.getParam("Ki",Ki);
    nh.getParam("Kd",Kd);

    cout<<"Kp Setup: "<<Kp<<endl;
    cout<<"Ki Setup: "<<Ki<<endl;
    cout<<"Kd Setup: "<<Kd<<endl;

    cout<<"Subscriber Setup"<<endl;

    des_pos_subscriber = nh.subscribe("/des_pos",1,&one_actuator::CallbackDesPos,this);
    encoder_subscriber = nh.subscribe("/encoder_data",1,&one_actuator::CallbackEncPos,this);
    
    cout<<"Subscribe to Desired Pos Topic : "<<endl;
    cout<<"Subscribe to Encoder Pos Topic : "<<endl;

    cout<<"Get Current Time"<<endl;
    current_time = ros::Time::now().toSec();
    last_time = current_time;
    dt = 1/50.0;
    
}

void one_actuator::CallbackDesPos(const Int32 & des_pos_msg)
{
    des_pos = des_pos_msg.data;
}

void one_actuator::CallbackEncPos(const Int32 & enc_pos_msg)
{
    current_time = ros::Time::now().toSec();
    dt = current_time - last_time;
    enc_pos = enc_pos_msg.data;

    if (isnan(dt) == true  || dt > 0.025)
        dt = 0.02;

    PID_control();

    last_time = current_time;
}

void one_actuator::PID_control()
{
    pos_error = (double) (des_pos - enc_pos);
    dpos_error = ((double) pos_error - (double) prev_pos_error)/dt;
    Ipos_error = (double)Ipos_error + dt*(double)pos_error;

    control_input = Kp*pos_error + Ki*Ipos_error + Kd*dpos_error;


    //cout<<"pos error: "<<pos_error<<"  ";
    //cout<<"des pos  : "<<des_pos<<endl;
    
    cout<<"Control Input : "<<control_input<<endl;
    
    // Control Direction
    if( control_input > 0)
        control_input_direction = 0;
    else
        control_input_direction = 1;

    // PWM Clamping
    if (abs(control_input) > max_pwm)
        control_input = max_pwm;


    digitalWrite(Pin_DIR,control_input_direction);
    pwmWrite(Pin_PWM,abs(control_input));

}

one_actuator::~one_actuator()
{
    des_pos_subscriber.~Subscriber();
    encoder_subscriber.~Subscriber();
}
