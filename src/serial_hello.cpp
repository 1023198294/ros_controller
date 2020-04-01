#include<iostream>
#include<ros/ros.h>
#include<stdlib.h>
#include<boost/asio.hpp>
#include<boost/bind.hpp>
#include<ros/spinner.h>
#include<std_msgs/String.h>
#include<mybot/Add.h>
using namespace std;
using namespace boost::asio;
int main(int argc,char** argv){
    ros::init(argc,argv,"serial_hello");
    ros::NodeHandle n;
    int hz;
    n.param<int>("hz",hz,2);
    ros::Rate loop_rate(hz);
    ROS_INFO_STREAM("ros initial COMPLETE");
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/chatter",1000);
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello ros";
    msg.data = ss.str();
    while(ros::ok()){
        chatter_pub.publish(msg);
        ROS_INFO("PUBLISH  hello ros DONE");
        ros::spinOnce();
        loop_rate.sleep();
    }
}
