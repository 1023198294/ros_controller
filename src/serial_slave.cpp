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
    ros::init(argc,argv,"serial_slave");
    ros::NodeHandle n;
    int x = 5;
    int y = 9;
    int baud,time_out,hz;
    n.param<int>("baud",baud,115200);
    n.param<int >("time_out",time_out,1000);
    n.param<int>("hz",hz,500);

    ROS_INFO_STREAM("ros initial COMPLETE");
    //ros::Subscriber SOURCE_sub = n.subscribe(topic,10,callback);
    //ros::ServiceServer service = n.advertiseService("Add",add);
    ros::Rate loop_rate(hz);
    ros::ServiceClient client = n.serviceClient<mybot::Add>("Add");
    mybot::Add srv;
    //srv.request.x = atoll(x);
    srv.request.x =  x;
    // srv.request.y = atoll(y);
    srv.request.y = y;
    if (client.call(srv)){
        ROS_INFO("Sum: %ld",(long int)srv.response.sum);
    }else{
        ROS_ERROR("Failed");
        return 1;
    }

    return 0;
}
