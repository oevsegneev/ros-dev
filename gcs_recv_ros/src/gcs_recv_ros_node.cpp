/**
 * @file gcs_recv_node.cpp
 * @brief poly project
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

//mavros_msgs::Mavlink last_msg;
void to_cb(const mavros_msgs::Mavlink::ConstPtr& msg){
    ROS_INFO("gcs_recv: %d-%d", msg->sysid, msg->msgid);
    if(msg->msgid == 70){
        ROS_DEBUG_NAMED("gsc_recv","Control intercepted");
    }
}

int main(int argc, char **argv)
{
    ROS_INFO("Interceptor is online");
    ros::init(argc, argv, "gcs_recv_node");
    ros::NodeHandle nh;

    ros::Subscriber to_sub = nh.subscribe<mavros_msgs::Mavlink>
            ("mavlink/to", 10, to_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
