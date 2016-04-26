/**
 * @file poly_ros_node.cpp
 * @brief poly project
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <mavlink.h>
//#include <pluginlib/class_list_macros.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/State.h>

#define LX 0
#define LY 1
#define LZ 2
#define LY 3

float local[4];

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

//mavros_msgs::Mavlink last_msg;
void to_px_cb(const mavros_msgs::Mavlink::ConstPtr& rmsg){
    mavlink_msg_set_position_target_local_ned_t pos_ned;
    mavlink_message_t mmsg;

    ROS_INFO("gcs_recv: %d-%d", rmsg->sysid, rmsg->msgid);
    if(rmsg->msgid == 84){
        mavros_msgs::mavlink::convert(*rmsg, mmsg); // from mavlink to ros
        mavlink_msg_set_position_target_local_ned_decode(mmsg, &pos_ned); // decode mavlink to struct
        ROS_INFO("pos_ned: %f,%f,%f,%f", pos_ned.x, pos_ned.y, pos_ned.z, pos_ned.yaw);
    }
}

void send_local_pos(){
    auto rmsg = boost::make_shared<mavros_msgs::Mavlink>();
    mavlink_message_t mmsg;

    mavlink_msg_local_position_ned_pack(1, 0, &mmsg, ros::Time::now(), 
                                        local[LX], local[LY], local[LZ],
                                        0, 0, 0);

    //rmsg->header.stamp = ros::Time::now();
    mavros_msgs::mavlink::convert(*mmsg, *rmsg);

    to_gcs.publish(rmsg);
    ROS_INFO("message has been sent to gcs);
}

int main(int argc, char **argv)
{
    ROS_INFO("Interceptor is online");
    ros::init(argc, argv, "gcs_recv_node");
    ros::NodeHandle nh;

    ros::Subscriber to_px = nh.subscribe<mavros_msgs::Mavlink>
            ("mavlink/to", 10, to_px_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher to_gcs = nh.subscribe<mavros_msgs::Mavlink>
            ("mavlink/from", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        send_local_pos();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
