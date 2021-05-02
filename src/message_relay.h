#ifndef MESSAGE_RELAY_H
#define MESSAGE_RELAY_H

#include <ros/ros.h>

#include <hexapod_msgs/LegsJoints.h>

class MessageRelay {
public:
    /// Creates an instance of MessageRelay
    /// \param nh The ROS node handler.
    MessageRelay(ros::NodeHandle *nh);

    /// Callback for converting hexapod_msgs::LegsJoints into std_msgs/Float64MultiArray and publishes the message
    /// to topic used by joint_group_position_controller in gazebo.
    /// \param legs_joints A reference to the LegJoints message.
    void send_to_joint_group_position_callback(const hexapod_msgs::LegsJoints::ConstPtr &legs_joints);
private:
    ros::Subscriber joints_command_subscriber_;
    ros::Publisher joints_command_gazebo_publisher_;
};


#endif //MESSAGE_RELAY_H
