#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "control_msgs/JointJog.h"
#include "sensor_msgs/JointState.h"
#include <string>
#include <sstream>

float joint[] = {0.0,0.0,0.0,0.0,0.0,0.0};
std::string joint_name[] = {"joint1","joint2","joint3","joint4","joint5","joint6"};
float demand_joint[] = {0.0,0.0,0.0,0.0,-0.09,0.0};
bool know_states;

class joint_controller
{
public:
    std::string joint_name;
    int joint_num;
public:
    joint_controller(int joint_number);
    float loop();
    ~joint_controller();
};

joint_controller::joint_controller(int joint_number)
{
    joint_name = "joint"+ std::to_string(joint_number);
    joint_num = joint_number;
}

float joint_controller::loop()
{
    float requested_angle = demand_joint[joint_num-1] + 1.57;
    float current_angle = joint[joint_num-1] + 1.57; 
    float signal = 0.35*(requested_angle-current_angle)/abs(requested_angle-current_angle); 
    if (abs(requested_angle - current_angle) > 0.04)
        return signal;
    else if (abs(requested_angle - current_angle) < 0.04)
        return 0;
}

joint_controller::~joint_controller()
{
}

void jointsCallback(const sensor_msgs::JointState::ConstPtr& msg) {
	int i = 0;
	for(std::vector<double>::const_iterator it = msg->position.begin(); it != msg->position.end(); ++it) {
        joint[i] = *it;
		i++;
	}
	know_states=true;
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
    ros::Subscriber joint_state_subscriber = node.subscribe("/joint_states", 1000, jointsCallback);
    ros::Subscriber joint_demand_subscriber = node.subscribe("/joint_demand", 1000, jointsDemandCallback);
    ros::Publisher joint_publisher = node.advertise<control_msgs::JointJog>("/JointJog",1);

    joint_controller j1(1),j2(2),j3(3),j4(4),j5(5),j6(6);
    ros::Rate loop_rate(30);
    while (ros::ok())
    {   
        float signal[] = {0.0,0.0,0.0,0.0,0.0,0.0};
        signal[0] =  j1.loop();
        signal[1] =  j2.loop();
        signal[2] =  j3.loop();
        signal[3] =  j4.loop();
        signal[4] =  j5.loop();
        signal[5] =  j6.loop();
        control_msgs::JointJog msg;
        for (int i = 1; i < 7; i++)
        {
            msg.joint_names.push_back(joint_name[i-1]);
            msg.velocities.push_back(signal[i-1]);   
        }
        msg.duration=5;
        joint_publisher.publish(msg);
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
