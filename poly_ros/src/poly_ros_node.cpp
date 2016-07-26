/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <cmath> 
#include <pluginlib/class_list_macros.h>
#include <mavros/utils.h>
#include <mavros/mavros_plugin.h>
#include <fstream>
#include <string>
#include <ctime>
//#include <ros/console.h>
#include <tuple>

#define LX 0
#define LY 1
#define LZ 2

#define YAW 0
#define PITCH 1
#define ROLL 2

float local[3];
float attitude[3];
float target_coord[3];
float target_att[3];
bool isRotate = false;
bool isMove = false;
bool xAxisMove = false;
std::string fileName = "/home/ubuntu/catkin_ws/src/poly_ros/log ";

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


// rad
double calc_angle()
{
    time_t rawTime;
    struct tm* timeInfo;
    time(&rawTime);
    timeInfo = std::localtime(&rawTime);
    std::ofstream fout;
    fout.open("/home/ubuntu/catkin_ws/src/poly_ros/log.txt", std::ios::app);
    double dx = target_coord[LX]-local[LX];
    double dy = target_coord[LY]-local[LY];
    if (fabs(dx)>fabs(dy)) xAxisMove = true;
    else xAxisMove = false;
    double a;
    /*if (fabs(dx)<0.001) a = M_PI/2.0;
    else*/ a = atan2(dx,dy);
    //else a = atan2(dy,dx); // ==dy/dx
    double angle;
    //if (local[LX] < target_coord[LX] && local[LY] < target_coord[LY]) angle = -M_PI/2.0 + a;
    //else if (local[LX] < target_coord[LX] && local[LY] > target_coord[LY]) angle = -M_PI/2.0 - a;
    //else if (local[LX] > target_coord[LX] && local[LY] < target_coord[LY]) angle = M_PI/2.0 - a;
    //else if (local[LX] > target_coord[LX] && local[LY] > target_coord[LY]) angle = M_PI/2.0 + a;
    if (local[LX] < target_coord[LX] && local[LY] < target_coord[LY]) angle = -a;
    else if (local[LX] < target_coord[LX] && local[LY] > target_coord[LY]) angle = -a;
    else if (local[LX] > target_coord[LX] && local[LY] < target_coord[LY]) angle = -a;
    else if (local[LX] > target_coord[LX] && local[LY] > target_coord[LY]) angle = -a;
    ROS_DEBUG("TX = %f, TY = %f, LX = %f, LY = %f, A = %f, LA = %f", 
    target_coord[LX],target_coord[LY], local[LX],local[LY],angle,attitude[YAW]);
    //std::string deb = "TX = {0}, TY = {1}, LX = {2}, LY = {3}, A = {4}, LA = {5}".format(target_coord[LX],target_coord[LY], local[LX],local[LY],angle,attitude[YAW]);
    //*asctime(timeInfo) + "\n"+
    char test[50];
    std::asctime(timeInfo);
    std::string deb = std::to_string(timeInfo->tm_hour)+":"+std::to_string(timeInfo->tm_min)+":"+std::to_string(timeInfo->tm_sec) + "\n"+
        " TX = " + std::to_string(target_coord[LX]) + "\n TY = " + std::to_string(target_coord[LY])
        + "\n LX = " + std::to_string(local[LX]) + "\n LY = " + std::to_string(local[LY])
        + "\n dx = " + std::to_string(dx) + "\n dy = " + std::to_string(dy) + "\n atan2 = " + std::to_string(a)
        + "\n TA = " + std::to_string(angle) + "\n LA = " + std::to_string(attitude[YAW])+"\n\n\n";
    fout << deb;
    fout.close();
    return angle;
}


//mavros_msgs::Mavlink last_msg;
void to_px_cb(const mavros_msgs::Mavlink::ConstPtr& rmsg){
    mavlink_set_position_target_local_ned_t pos_ned;
    mavlink_message_t mmsg;

    ROS_INFO("gcs_recv: %d-%d", rmsg->sysid, rmsg->msgid);
    if(rmsg->msgid == 84){
        mavros_msgs::mavlink::convert(*rmsg, mmsg); // from mavlink to ros
        mavlink_msg_set_position_target_local_ned_decode(&mmsg, &pos_ned); // decode mavlink to struct
        target_coord[LX] = pos_ned.x;
        target_coord[LY] = pos_ned.y;
        target_coord[LZ] = pos_ned.z;
        target_att[YAW] = calc_angle();
        //target_att[YAW] = pos_ned.yaw;
        isRotate = true;
        ROS_INFO("pos_ned: %f,%f,%f,%f", pos_ned.x, pos_ned.y, pos_ned.z, pos_ned.yaw);
    }
}

std::tuple<double,double> turn90(double x, double y)
{
    // transform degree to radian
    double lx = cos(-M_PI/2)*x - sin(-M_PI/2)*y;
    double ly = sin(-M_PI/2)*x + cos(-M_PI/2)*y;
    return std::make_tuple (-lx, -ly); // change sign on X axis for CS reflection, trouble with Y-axis (wrote test "-")
}

void lidar_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    std::tuple<double, double> turnCoord = turn90(msg->pose.position.x, msg->pose.position.y);
    //local[LX] = std::get<0>(turnCoord);
    local[LX] = msg->pose.position.x;
    //local[LY] = std::get<1>(turnCoord);
    local[LY] = msg->pose.position.y;
    local[LZ] = msg->pose.position.z;
    attitude[YAW] = msg->pose.orientation.z*M_PI;
    //double z_coord = atan2(2*(q1*q4+q2*q3), 1-2*(q3*q3+q4*q4));
}

int send_local_pos(mavros_msgs::Mavlink *rmsg){
    mavlink_message_t mmsg;

    mavlink_msg_local_position_ned_pack(1, 10, &mmsg, ros::Time::now().toNSec() / 1000, local[LX], local[LY], local[LZ],0, 0, 0);
    mavros_msgs::mavlink::convert(mmsg, *rmsg);

    return 1;
}

int send_attitude(mavros_msgs::Mavlink *rmsg){
    mavlink_message_t mmsg;

    mavlink_msg_attitude_pack(1, 10, &mmsg, ros::Time::now().toNSec() / 1000, attitude[ROLL], attitude[PITCH], attitude[YAW],0,0,0);
    mavros_msgs::mavlink::convert(mmsg, *rmsg);

    return 1;
}

int rotate_function()
{
    ROS_INFO("target angle %f ", target_att[YAW]);
    if (fabs(attitude[YAW]-target_att[YAW])<0.15) // 10 degree on right and 10 degree on left
    {
        isRotate = false;
        isMove = true;
        ROS_INFO("Ready to move");
        return 1500;
    }
    // protiv
    else if (attitude[YAW]<target_att[YAW])
    {
        //double k = 100;
        //double ang = fabs(attitude[YAW]-target_att[YAW]);
        ROS_INFO("left");
        //return ang>1 ? (1500-ang*k) : (1500-100);
        return 1180;
    }
    // po
    else if (attitude[YAW]>target_att[YAW])
    {
        //double k = 100;
        //double ang = fabs(attitude[YAW]-target_att[YAW]);
        ROS_INFO("right");
        //return ang>1 ? (1500+ang*k) : (1500+100);
        return 1820;
    }
}

int move_function()
{
    // must been broken, when trouble with angle: || instead of &&
    if (xAxisMove && fabs(local[LX]-target_coord[LX])<0.02 ||  
        !xAxisMove && fabs(local[LY]-target_coord[LY])<0.02)
    {
        ROS_INFO("In place. %f, %f, %f, %f", local[LX], target_coord[LX], local[LY], target_coord[LY]);
        isMove = false;
        return 1500;
    }
    // forward
    else
    {
        //isRotate = true;
        //isMove = false;
        ROS_INFO("Moving. %f, %f", local[LX]- target_coord[LX], local[LY]- target_coord[LY]);
        //ROS_INFO("Moving");
        return 1600;
    }
}

int main(int argc, char **argv)
{
    ROS_INFO("Poly on");
    ROS_DEBUG("Poly on debug");
    
    //std::ofstream fout;
    //fout.open("/home/ubuntu/catkin_ws/src/poly_ros/log.txt", std::ios::trunc);
    //fout.close();
    ROS_DEBUG_STREAM("Poly on debug stream");
    ros::init(argc, argv, "poly_ros_node");
    ros::NodeHandle nh;

    ros::Subscriber to_px = nh.subscribe<mavros_msgs::Mavlink>
            ("mavlink/to", 10, to_px_cb);
    ros::Subscriber lidar_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("slam_out_pose", 10, lidar_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>
            ("mavros/rc/override", 1);
    ros::Publisher to_gcs = nh.advertise<mavros_msgs::Mavlink>
            ("mavlink/from", 10);
    //ros::Publisher local_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
    //        ("mavlink/form", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
       rate.sleep();
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "MANUAL";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    mavros_msgs::OverrideRCIn rc;
    geometry_msgs::PoseStamped local_pose;
    mavros_msgs::Mavlink rmsg;

    while(ros::ok()){
        if( current_state.mode != "MANUAL" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Manual enabled");
                rc.channels[2]=1570;
                rc_pub.publish(rc);
                ros::Rate initRate(1.0);
                initRate.sleep();
                rc.channels[2]=1430;
                rc_pub.publish(rc);
                initRate.sleep();
            }
            last_request = ros::Time::now();
        }
            /*if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }*/
        
        /*if( current_state.mode != "MANUAL"){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Manual enabled");
            }
        }*/
        if(send_local_pos(&rmsg))
            to_gcs.publish(rmsg);
        if(send_attitude(&rmsg))
            to_gcs.publish(rmsg);
        if (isRotate)
        {
            rc.channels[0] = rotate_function();
            rc.channels[2] = 1500;
        }
        else if (isMove)
        {
            //ROS_INFO("Go forward");
            rc.channels[0] = 1500;
            rc.channels[2] = move_function();
            // don't delete
            /*if ((!xAxisMove && fabs(local[LX]-target_coord[LX])>0.07 || 
                xAxisMove && fabs(local[LY]-target_coord[LY])>0.07) && fabs(attitude[YAW]-target_att[YAW])>0.3) 
            {
                target_att[YAW]=calc_angle();
                isRotate = true;
                rc.channels[2]=1500;
            }*/
        }
        else
        {
            rc.channels[0] = 1500;
            rc.channels[2] = 1500;
        }

        rc_pub.publish(rc);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
