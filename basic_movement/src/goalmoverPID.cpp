#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "control_msgs/JointJog.h"
#include "sensor_msgs/JointState.h"
#include <string>
#include <sstream>

float joint[] = {0.0,0.0,0.0,0.0,0.0,0.0};
float demand_joint[] = {0.0,0.0,0.0,0.0,0.0,0.0};

class joint_controller
{
public:
    std::string joint_name;
    float kp;
    float kd;
    float ki;
    float max_vel;
    float min_vel;
    float previourEr;
    float integral;
    joint_controller();
    joint_controller(std::string name, float Kp, float Ki, float Kd, float maxVel, float minVel);
    float stepPID(float step, float joint_demand, float joint_current);
    float stepBB(float joint_demand, float joint_current);
};

joint_controller::joint_controller() {}

joint_controller::joint_controller(std::string name, float Kp, float Ki, float Kd, float maxVel, float minVel)
{
    joint_name = name;
    kp = Kp;
    ki = Ki;
    kd = Kd;
    max_vel = maxVel;
    min_vel = minVel;
    previourEr = 0.0;
}

float joint_controller::stepBB(float joint_demand, float joint_current)
{
    float requested_angle = joint_demand + 1.57;
    float current_angle = joint_current + 1.57; 
    float signal = 0.35*(requested_angle-current_angle)/abs(requested_angle-current_angle); 
    if (abs(requested_angle - current_angle) > 0.04)
        return signal;
    else if (abs(requested_angle - current_angle) < 0.04)
        return 0;
}

float joint_controller::stepPID(float step, float joint_demand, float joint_current)
{
    float requested_angle = joint_demand + 1.57;
    float current_angle = joint_current + 1.57; 
    float err = requested_angle - current_angle;    // Calculate error

    float PComponent = kp * err;    // proportional Component

    integral += err * step;
    float IComponent = ki * integral;   // integral Component

    float derivative = (err - previourEr) / step;
    float DComponent = kd * derivative;     // derivative Component

    float control_signal = PComponent + IComponent + DComponent;   // calculate total output

    if( control_signal > max_vel )  // max velocity capping
        control_signal = max_vel;
    else if( control_signal < min_vel ) // min velocity capping
        control_signal = min_vel;
    
    previourEr = err;

    return control_signal;
}

void jointsCallback(const sensor_msgs::JointState::ConstPtr& msg) {
	int i = 0;
	for(std::vector<double>::const_iterator it = msg->position.begin(); it != msg->position.end(); ++it) {
        joint[i] = *it;
		i++;
	}
}

void jointsDemandCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    int i = 0;
 	for(std::vector<float>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
	{
		demand_joint[i] = *it;
		i++;
	}   
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_movement");
	ros::NodeHandle node;
    ros::Subscriber joint_state_subscriber = node.subscribe("/joint_states", 1, jointsCallback);
    ros::Subscriber joint_demand_subscriber = node.subscribe("/joint_demand", 1, jointsDemandCallback);
    ros::Publisher joint_publisher = node.advertise<control_msgs::JointJog>("/JointJog",1);

    joint_controller* joint_array = new joint_controller[6];

    joint_array[0] = joint_controller("joint1", 0.1, 0.001 , 0.0005 , 0.35 , -0.35 );
    joint_array[1] = joint_controller("joint2", 0.1, 0.001 , 0.0005 , 0.35 , -0.35 );
    joint_array[2] = joint_controller("joint3", 0.1, 0.001 , 0.0005 , 0.35 , -0.35 );
    joint_array[3] = joint_controller("joint4", 0.1, 0.001 , 0.0005 , 0.35 , -0.35 );
    joint_array[4] = joint_controller("joint5", 0.1, 0.001 , 0.0005 , 1.00 , -1.00 );
    joint_array[5] = joint_controller("joint6", 0.1, 0.001 , 0.0005 , 0.35 , -0.35 );
    
    ros::Rate loop_rate(30);    //controller frequencies
    ros::Time stepTime = ros::Time::now();
    while (ros::ok())
    {
        control_msgs::JointJog msg;
        for (int i = 0; i <= 5; i++)
        {
            msg.joint_names.push_back(joint_array[i].joint_name);   //joint name
            float BBContOut = joint_array[i].stepBB(demand_joint[i], joint[i]);
            float PIDContOut = joint_array[i].stepPID((ros::Time::now().toSec()-stepTime.toSec()), demand_joint[i], joint[i]);
            msg.velocities.push_back(PIDContOut);   //control signal to the joint
        }
        joint_publisher.publish(msg);
        stepTime = ros::Time::now();
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
