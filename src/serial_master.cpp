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
void callback(const std_msgs::String::ConstPtr& msg);
bool add(mybot::Add::Request &req
,mybot::Add::Response &res){
    res.sum = req.x+req.y;
    ROS_INFO("done! x=%ld,y=%ld,sum=%ld",(long int)req.x,(long int)req.y,(long int)res.sum);
    return true;
}
int main(int argc,char** argv){
    ros::init(argc,argv,"serial_master");
    ros::NodeHandle n;
    int baud,time_out,hz;
    string topic;
    n.param<int>("baud",baud,115200);
    n.param<int >("time_out",time_out,1000);
    n.param<int>("hz",hz,500);
    n.param<std::string>("topic",topic,"/chatter");
    ROS_INFO_STREAM("ros initial COMPLETE");
    ros::Subscriber sub_chatter = n.subscribe(topic,10,callback);
    ros::ServiceServer service = n.advertiseService("Add",add);
    ros::Rate loop_rate(hz);
    while(ros::ok()){

           //ROS_INFO_STREAM("PLEASE MONEY!");
            ros::spinOnce();
            loop_rate.sleep();
            }
return 0;
}
void callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("I have heard %s",msg->data.c_str());
}