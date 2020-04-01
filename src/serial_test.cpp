#include<iostream>
#include<string>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sstream>
#include <vector>

#include <ros/ros.h> 
#include <ros/spinner.h>
#include<boost/bind.hpp>
#include<boost/asio.hpp>
#include <geometry_msgs/Twist.h>
#include<sys/time.h>
#include <serial/serial.h> 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include<tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define STS  0x51
#define EXT  0x52
#define  CMD_QSETUP  0x01
#define CMD_QGET 0x03
#define CMD_QSEND 0x05
#define CMD_QOPR  0x07

#define SER_MISS 0x00
#define SER_WAIT 0x01
#define SER_OK 0x02
#define SER_WR 0x03
#define  SER_RC 0x04


using namespace std;
using namespace boost::asio;

serial::Serial ros_ser;
int current_state,rnd;
float rx_buffer[4];

void analy_uart_receive_data(float datas[],std_msgs::String serial_data);
void serial_hang(int size);
void send_CarBus(uint8_t cmd,float fdata1,float fdata2,float fdata3,float fdata4, int rnd);
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
void calculate_position_by_odometry(void);
void publish_odometry(float positionX,float positionY,float orientation,float linearX,float linearY,float linearW);
float WHEEL_RATIO =19.0; 		//减速比
float WHEEL_K=0.436;                              // abs(X) + abs(Y)
float WHEEL_D=0.1525; 			//轮子直径
float WHEEL_R=WHEEL_D/2.0; 			//轮子半径
float WHEEL_PI=3.141693; 			//pi
float K4 = 1.0/(4.0*WHEEL_K); //参数
float DELTA_T;
int exec_mode = 1;
float speedX,speedY,speedW; //三个方向的分速度
float v1=0,v2=0,v3=0,v4=0; //对应四个轮速
float s1=0,s2=0,s3=0,s4=0;//对应四轮累计偏移
float s1_last=0,s2_last=0,s3_last=0,s4_last=0;//对应四轮上一时刻累计偏移

int rpm1,rpm2,rpm3,rpm4;//对应四个轮子的rpm
int angle1,angle2,angle3,angle4;//对应四个轮子的转角
int rnd1,rnd2,rnd3,rnd4;//对应四个轮子的圈数
float positionX,positionY,positionW;//对应里程计坐标


ros::Publisher odom_pub;

int main (int argc, char** argv) 
{ 
    std_msgs::UInt8MultiArray r_buffer;
    int buad,time_out,hz;
    string dev,pub_odom_topic;    
    ros::init(argc, argv, "serial_test");
    ros::NodeHandle n; 
    n.param<int>("baud",buad,115200);
    n.param<int >("time_out",time_out,1000);
    n.param<int>("hz",hz,50);
    n.param<std::string>("pub_odom_topic",pub_odom_topic,"/car_odom");
    n.param<std::string>("dev",dev,"/dev/ttyUSB0");
    DELTA_T = 1.0/hz;
    ros::Rate loop_rate(hz);
    odom_pub = n.advertise<nav_msgs::Odometry>(pub_odom_topic,20);
    ros::Subscriber command_sub = n.subscribe("turtle1/cmd_vel",10,cmd_vel_callback);
    try{    
        ros_ser.setPort(dev);
        ros_ser.setBaudrate(buad);
        serial::Timeout to = serial::Timeout::simpleTimeout(time_out);
        ros_ser.setTimeout(to);
        ros_ser.open();
        ros_ser.flushInput(); //清空缓冲区数据
    }
    catch (serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port ");
        exec_mode = 0;
    }
    if(ros_ser.isOpen()){
        ros_ser.flushInput(); //清空缓冲区数据
        ROS_INFO_STREAM("Serial Port opened");
    }
    else{
        return -1;
    }

    rnd = 0;
    sleep(1);
    while(ros::ok()){  
        std_msgs::String serial_data;
        if(exec_mode == 1){
            if (!current_state){
                send_CarBus(0x01,0,0,0,0,rnd);
                rnd = (rnd+1)%100;
                serial_hang(21);
                
                serial_data.data = ros_ser.read(ros_ser.available());
                if (serial_data.data[1] == 0x02){
                    rnd = (serial_data.data[2]-'0')*10+(serial_data.data[3]-'0');
                    ROS_INFO("Initial Done,RND = %d",rnd);
                    current_state = 0x01;
                } 
            }
            
            
            
            /*
            //query data
            send_CarBus(0x03,1,0,0,0,rnd);
            rnd = (rnd+1)%100;
            serial_hang(21);
            serial_data.data = ros_ser.read(ros_ser.available());
            analy_uart_receive_data(rx_buffer,serial_data);
            ROS_INFO("Receive Data %f %f %f %f Done,RND = %d",rx_buffer[0],rx_buffer[1],rx_buffer[2],rx_buffer[3],rnd);
            */
            
            //query data
            send_CarBus(0x03,2,0,0,0,rnd);
            rnd = (rnd+1)%100;
            serial_hang(21);
            serial_data.data = ros_ser.read(ros_ser.available());
            analy_uart_receive_data(rx_buffer,serial_data);
            //ROS_INFO("Receive RPM %f %f %f %f Done,RND = %d",rx_buffer[0],rx_buffer[1],rx_buffer[2],rx_buffer[3],rnd);
            rpm1 = rx_buffer[0];
            rpm2 = rx_buffer[1];
            rpm3 = rx_buffer[2];
            rpm4 = rx_buffer[3];
            
            
            //query data
            send_CarBus(0x03,3,0,0,0,rnd);
            rnd = (rnd+1)%100;
            serial_hang(21);
            serial_data.data = ros_ser.read(ros_ser.available());
            analy_uart_receive_data(rx_buffer,serial_data);
            //ROS_INFO("Receive ANG %f %f %f %f Done,RND = %d",rx_buffer[0],rx_buffer[1],rx_buffer[2],rx_buffer[3],rnd);
            angle1 = (int)rx_buffer[0]%8192;
            angle2 = (int)rx_buffer[1]%8192;
            angle3 = (int)rx_buffer[2]%8192;
            angle4 = (int)rx_buffer[3]%8192;


            //query data
            send_CarBus(0x03,4,0,0,0,rnd);
            rnd = (rnd+1)%100;
            serial_hang(21);
            serial_data.data = ros_ser.read(ros_ser.available());
            analy_uart_receive_data(rx_buffer,serial_data);
            //ROS_INFO("Receive RND %f %f %f %f Done,RND = %d",rx_buffer[0],rx_buffer[1],rx_buffer[2],rx_buffer[3],rnd);
            rnd1 = rx_buffer[0];
            rnd2 = rx_buffer[1];
            rnd3 = rx_buffer[2];
            rnd4 = rx_buffer[3];

            calculate_position_by_odometry();
        
            //send self data
            send_CarBus(0x05,v1,v2,v3,v4,rnd);
            rnd = (rnd+1)%100;
            serial_hang(21);
            serial_data.data = ros_ser.read(ros_ser.available());
            if (serial_data.data[1]==0x06){
                //ROS_INFO("Send Data Done,RND = %d",rnd);
            }else{
            //ROS_INFO("Send Data Failed,RND = %d",rnd);
            }
            
        }
        //float d1,d2,d3,d4 = analy_uart_receive_data(serial_data);
        ros::spinOnce();
        loop_rate.sleep();
    }
} 


void send_CarBus(uint8_t cmd,float fdata1,float fdata2,float fdata3,float fdata4, int rnd){
    uint8_t data_tem[21];
    int data1 = (int) fdata1;
    int data2 = (int) fdata2;
    int data3 = (int) fdata3;
    int data4 = (int) fdata4;
    data_tem[0] = 0x51;
	data_tem[20] = 0x52;
	data_tem[1] = cmd;
	data_tem[2] = rnd/10+'0';
	data_tem[3] = rnd%10+'0';
	data_tem[4] = data1/1000+'0';
	data_tem[5] = data1/100%10+'0';
	data_tem[6] = data1/10%10+'0';
	data_tem[7] = data1%10+'0';
	data_tem[8] = data2/1000+'0';
	data_tem[9] = data2/100%10+'0';
	data_tem[10] = data2/10%10+'0';
	data_tem[11] = data2%10+'0';
	data_tem[12] = data3/1000+'0';
	data_tem[13] = data3/100%10+'0';
	data_tem[14] = data3/10%10+'0';
	data_tem[15] = data3%10+'0';
	data_tem[16] = data4/1000+'0';
	data_tem[17] = data4/100%10+'0';
	data_tem[18] = data4/10%10+'0';
	data_tem[19] = data4%10+'0';
    ros_ser.write(data_tem,21);
}
void serial_hang(int size=21){
    while(ros_ser.available()<size){
        //ROS_INFO("%d",ros_ser.available());
        sleep(0.1);
    }
}   
void analy_uart_receive_data(float datas[],std_msgs::String serial_data){
    for (int i =0;i<4;i++){ 
        datas[i] =(serial_data.data[4*i+4] -'0')*1000+(serial_data.data[4*(i+1)+1]-'0')*100 + (serial_data.data[4*(i+1)+2]-'0')*10 + serial_data.data[4*(i+1)+3]-'0';
        if(datas[i]>30000){
            datas[i]-=65536;
        }
    }
}
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
   
    speedX = msg->linear.x*12;//乌龟键盘不够快！
    speedY = msg->linear.y*12;
    speedW = msg->angular.z*40;
    v4 = speedX-speedY-WHEEL_K*speedW;
    v2 = speedX+speedY-WHEEL_K*speedW;
    v1 = -(speedX-speedY+WHEEL_K*speedW);
    v3 = -(speedX+speedY+WHEEL_K*speedW);
    v1 = v1/(2.0*WHEEL_RATIO*WHEEL_PI);//线速度到角速度
    v2 = v2/(2.0*WHEEL_RATIO*WHEEL_PI);
    v3 =  v3/(2.0*WHEEL_RATIO*WHEEL_PI);
    v4 = v4/(2.0*WHEEL_RATIO*WHEEL_PI);
    
    v1 = v1*WHEEL_RATIO*60;
    v2 = v2*WHEEL_RATIO*60;
    v3 = v3*WHEEL_RATIO*60;
    v4 = v4*WHEEL_RATIO*60;

    //ROS_INFO("v1=%f v2=%f v3=%f v4=%f",v1,v2,v3,v4);
}

void calculate_position_by_odometry(void){
    float delta_s1,delta_s2,delta_s3,delta_s4;
    float delta_positionX,delta_positionY,delta_positionW;
    float linearX,linearY,linearW;//对应里程计速度
    float spd1,spd2,spd3,spd4;

    s1_last = s1;
    s2_last = s2;
    s3_last = s3;
    s4_last = s4;
    //s1 = spd1*DELTA_T;积分法不可靠哦
    //s2 = spd2*DELTA_T;
    //s3 = spd3*DELTA_T;
    //s4 = spd4*DELTA_T;
    s1 = (rnd1+angle1%8192/8192)/WHEEL_RATIO*WHEEL_PI*WHEEL_D;
    s2 = (rnd2+angle2%8192/8192)/WHEEL_RATIO*WHEEL_PI*WHEEL_D;
    s3 = (rnd3+angle3%8192/8192)/WHEEL_RATIO*WHEEL_PI*WHEEL_D;
    s4 = (rnd4+angle4%8192/8192)/WHEEL_RATIO*WHEEL_PI*WHEEL_D;
    delta_s1 = s1-s1_last;
    delta_s2 = s2-s2_last;
    delta_s3 = s3-s3_last;
    delta_s4 = s4-s4_last;
    delta_positionX = 0.25*delta_s1+0.25*delta_s2-0.25*delta_s3-0.25*delta_s4;
    delta_positionY = -0.25*delta_s1+0.25*delta_s2-0.25*delta_s3+0.25*delta_s4;
    delta_positionW = -K4*delta_s1-K4*delta_s2-K4*delta_s3-K4*delta_s4;//弧度

    positionX = positionX +  cos(positionW)*delta_positionX-sin(positionW)*delta_positionY;
    positionY = positionY +  sin(positionW)*delta_positionX + cos(positionW)*delta_positionY;
    positionW = positionW + delta_positionW;

    if (positionW>2*WHEEL_PI)
        positionW -= 2*WHEEL_PI;
    else if (positionW < -2*WHEEL_PI)
        positionW += 2*WHEEL_PI;
    spd1 = rpm1/60.0/WHEEL_RATIO*WHEEL_PI*WHEEL_D;
    spd2 = rpm2/60.0/WHEEL_RATIO*WHEEL_PI*WHEEL_D;
    spd3 = rpm3/60.0/WHEEL_RATIO*WHEEL_PI*WHEEL_D;
    spd4 = rpm4/60.0/WHEEL_RATIO*WHEEL_PI*WHEEL_D;
    linearX = 0.25*spd1+0.25*spd2-0.25*spd3-0.25*spd4;
    linearY = -0.25*spd1+0.25*spd2-0.25*spd3+0.25*spd4;
    linearW = -K4*spd1-K4*spd2-K4*spd3-K4*spd4;

    ROS_INFO("Position_X:%f Position_Y: %f Position_W: %f",positionX,positionY,positionW);
    //ROS_INFO("Linear_X: %f Linear_Y: %f Linear_W: %f",linearX,linearY,linearW);
}
void publish_odometry(float positionX,float positionY,float orientation,float linearX,float linearY,float linearW){
    static tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    geometry_msgs::Quaternion odom_quat;
    nav_msgs::Odometry odom;
    odom_quat = tf::createQuaternionMsgFromYaw(orientation);
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = positionX;
    odom_trans.transform.translation.y = positionY;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = positionX;
    odom.pose.pose.position.y = positionY;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    
    odom.twist.twist.linear.x = linearX;
    odom.twist.twist.linear.y = linearY;
    odom.twist.twist.angular.z = linearW;

    odom_pub.publish(odom);
}