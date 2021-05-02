#include <ros/console.h>
#include <std_msgs/Float64MultiArray.h>

#include "message_relay.h"


MessageRelay::MessageRelay(ros::NodeHandle *nh) {
    joints_command_gazebo_publisher_ = nh->advertise<std_msgs::Float64MultiArray>("joint_group_position_controller/command", 1);
    joints_command_subscriber_ = nh->subscribe("joints_command", 1, &MessageRelay::send_to_joint_group_position_callback, this);
}

void MessageRelay::send_to_joint_group_position_callback(const hexapod_msgs::LegsJoints::ConstPtr &legs_joints) {
    // Create message to be published
    std_msgs::Float64MultiArray joint_array_rad;
    // Assuming we only have 18 joints.
    joint_array_rad.data.resize(18);

    // The mapping of the array of which index corresponds to which joint is found in the config/yaml file.
    joint_array_rad.data[0] = legs_joints->left_front_leg.coxa;
    joint_array_rad.data[1] = legs_joints->left_front_leg.femur;
    joint_array_rad.data[2] = legs_joints->left_front_leg.tibia;

    joint_array_rad.data[3] = legs_joints->left_mid_leg.coxa;
    joint_array_rad.data[4] = legs_joints->left_mid_leg.femur;
    joint_array_rad.data[5] = legs_joints->left_mid_leg.tibia;

    joint_array_rad.data[6] = legs_joints->left_back_leg.coxa;
    joint_array_rad.data[7] = legs_joints->left_back_leg.femur;
    joint_array_rad.data[8] = legs_joints->left_back_leg.tibia;

    joint_array_rad.data[9] = legs_joints->right_front_leg.coxa;
    joint_array_rad.data[10] = legs_joints->right_front_leg.femur;
    joint_array_rad.data[11] = legs_joints->right_front_leg.tibia;

    joint_array_rad.data[12] = legs_joints->right_mid_leg.coxa;
    joint_array_rad.data[13] = legs_joints->right_mid_leg.femur;
    joint_array_rad.data[14] = legs_joints->right_mid_leg.tibia;

    joint_array_rad.data[15] = legs_joints->right_back_leg.coxa;
    joint_array_rad.data[16] = legs_joints->right_back_leg.femur;
    joint_array_rad.data[17] = legs_joints->right_back_leg.tibia;

    joints_command_gazebo_publisher_.publish(joint_array_rad);
}

int main(int argc, char **argv)
{
    ROS_INFO("Message relay started.");
    const std::string node_name = "message_relay";
    ros::init(argc, argv, node_name);
    ros::NodeHandle n("~");

    MessageRelay message_relay{&n};

    ros::spin();
    return 0;
}
