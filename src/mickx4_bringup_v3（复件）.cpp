#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <math.h>
#include <chrono>
#include <thread>

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "sensor_msgs/msg/imu.hpp"
// #include "tf2_ros/transform_broadcaster.h"
// #include "nav_msgs/msg/odometry.hpp"
// #include "nav_msgs/msg/path.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "sensor_msgs/msg/joy.hpp"
#include "serial/serial.h"
//#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include "eigen3/Eigen/Geometry"
#include "sys/time.h"
#include "chassis_mick_msg.h"

using namespace std;


 




 
void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
void send_speed_to_X4chassis(float x,float y,float w);
void send_rpm_to_chassis( int w1, int w2, int w3, int w4);
 

void clear_odometry_chassis(void);
bool analy_uart_recive_data(std_msgs::msg::String serial_data,
								rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub);
void calculate_position_for_odometry(void);
void calculate_chassisDiffX4_position_for_odometry(rclcpp::Node::SharedPtr node,
						rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub,
								rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub);
 
void publish_odomtery(rclcpp::Node::SharedPtr node,rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub,
						rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub,
							float position_x,float position_y,float oriention,float vel_linear_x,float vel_linear_y,float vel_linear_w);
 


int main(int argc, char ** argv)
{
    string out_result;
    bool uart_recive_flag;

    string sub_cmdvel_topic = "cmd_vel";
    string pub_odom_topic = "odom";
    string pub_imu_topic = "Imu";
 

    rclcpp::init(argc,argv);

    auto node = std::make_shared<rclcpp::Node>("mick_robot");

    node->declare_parameter<std::string>("sub_cmdvel_topic",sub_cmdvel_topic);
    node->declare_parameter<std::string>("pub_odom_topic",pub_odom_topic);
    node->declare_parameter<std::string>("pub_imu_topic",pub_imu_topic);
    node->declare_parameter<std::string>("dev",dev);

    node->declare_parameter<int>("baud",baud);
    node->declare_parameter<int>("time_out",time_out);
    node->declare_parameter<int>("hz",hz);
    node->declare_parameter<int>("is_pub_path",is_pub_path);
    node->declare_parameter<int>("chassis_type",chassis_type);

    node->declare_parameter<std::string>("joy_topic",joy_topic);
    node->declare_parameter<float>("RC_K",RC_K);
    node->declare_parameter<float>("RC_MIN",RC_MIN);
    node->declare_parameter<float>("RC_MAX",RC_MAX);

	RCLCPP_INFO_STREAM(node->get_logger(),"sub_cmdvel_topic:   "<<sub_cmdvel_topic);
	RCLCPP_INFO_STREAM(node->get_logger(),"pub_odom_topic:   "<<pub_odom_topic);
	RCLCPP_INFO_STREAM(node->get_logger(),"pub_imu_topic:   "<<pub_imu_topic);
 
	RCLCPP_INFO_STREAM(node->get_logger(),"RC_K:   " << RC_K);
	RCLCPP_INFO_STREAM(node->get_logger(),"RC_MIN:   " << RC_MIN);
	RCLCPP_INFO_STREAM(node->get_logger(),"RC_MAX:   " << RC_MAX);

	auto command_sub = node->create_subscription<geometry_msgs::msg::Twist>(sub_cmdvel_topic, 10, cmd_vel_callback);

    auto joy_pub = node->create_publisher<sensor_msgs::msg::Joy>(joy_topic,20);
    auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>(pub_imu_topic,rclcpp::QoS(20).transient_local());
    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(pub_odom_topic,20);
    auto path_pub = node->create_publisher<nav_msgs::msg::Path>(pub_odom_topic+"/path",rclcpp::QoS(20).transient_local());

 

    rclcpp::Rate loop_rate(hz);

    while (rclcpp::ok())
    {
		if(ros_ser.available() )
		{
			//ROS_INFO_STREAM("Reading from serial port");
			std_msgs::msg::String serial_data;
			//获取串口数据
			serial_data.data = ros_ser.read(ros_ser.available());
			//cout<<serial_data.data << "\n"<<endl;
			uart_recive_flag = analy_uart_recive_data(serial_data,imu_pub);
			if(uart_recive_flag)
			{
				uart_recive_flag=0;
				if(chassis_type == 0 || chassis_type == 1)
				{
					calculate_chassisDiffX4_position_for_odometry(node,odom_pub,path_pub);
					//calculate_position_for_odometry();
				}
				else
				{
					;
				}
			    // odom_pub.publish(serial_data);//将串口数据发布到主题sensor
			}
			else
			{
				RCLCPP_INFO_STREAM(node->get_logger()," analy uart recive data error ...");
				//serial_data.data = ros_ser.read(ros_ser.available());
				ros_ser.flushInput(); //清空缓冲区数据
				//sleep(0.2);            //延时0.2秒,确保有数据进入
				std::this_thread::sleep_for(std::chrono::milliseconds(200));

			}
		}
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    std::cout<<" EXIT ..."<<std::endl;
    rclcpp::shutdown();
    return 0;
}


/**
 * @function 解析串口发送过来的数据帧
 * 成功则返回true　否则返回false
 */
bool analy_uart_recive_data(std_msgs::msg::String serial_data,
								rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub)
{
	unsigned char reviced_tem[500];
	uint16_t len=0,i=0,j=0;
	unsigned char check=0;
	unsigned char tem_last=0,tem_curr=0,rec_flag=0;//定义接收标志位
	uint16_t header_count=0,step=0; //计数这个数据序列中有多少个帧头
	len=serial_data.data.size();
	if(len<1 || len>500)
	{
		cout<<"serial data is too short ,  len: " <<serial_data.data.size() <<endl;
		std_msgs::msg::String serial_data;
		string str_tem;

		serial_data.data = ros_ser.read(ros_ser.available());
		str_tem =  serial_data.data;
		return false; //数据长度太短　
	}
	//ROS_INFO_STREAM("Read: " << serial_data.data.size() );

	// 有可能帧头不在第一个数组位置
	for( i=0;i<len;i++) 
	{
		tem_last=  tem_curr;
		tem_curr = serial_data.data.at(i);
		if(tem_last == 0xAE && tem_curr==0xEA&&rec_flag==0) //在接受的数据串中找到帧头　
		{
			rec_flag=1;
			reviced_tem[j++]=tem_last;
			reviced_tem[j++]=tem_curr;
			//ROS_INFO_STREAM("found frame head" ); 
		}
		else if (rec_flag==1)
		{
			reviced_tem[j++]=serial_data.data.at(i);
			if(tem_last == 0xEF && tem_curr==0xFE)
			{
				header_count++;
				rec_flag=0;
			}
		}
		else
			rec_flag=0;
	}
	// 检验数据长度和校验码是否正确
	//   if(reviced_tem[len-3] ==check || reviced_tem[len-3]==0xff)
	//     ;
	//   else
	//     return;
	// 检验接受数据的长度
	step=0;
	for(int k=0;k<header_count;k++) 
	{
		len = (reviced_tem[2+step] +4 ) ; //第一个帧头的长度
		//cout<<"read head :" <<i<< "      len:   "<<len;
		if(reviced_tem[0+step] ==0xAE && reviced_tem[1+step] == 0xEA && reviced_tem[len-2+step]==0xEF &&reviced_tem[len-1+step]==0xFE) 
		{//检查帧头帧尾是否完整
			if (reviced_tem[3+step] ==0x07 ) //mickv3 4个电机 数据
			{
				i=4+step;
				for(int j=0;j<4;j++)
				{
					speed_rpm.int32_dat=0;
					total_angle.int32_dat =0;
					round_cnt.int32_dat=0;

					speed_rpm.byte_data[3]=reviced_tem[i++]; 
					speed_rpm.byte_data[2]=reviced_tem[i++];
					speed_rpm.byte_data[1] = reviced_tem[i++] ; 
					speed_rpm.byte_data[0] = reviced_tem[i++] ;
					moto_chassis[j].speed_rpm = speed_rpm.int32_dat;			//*1000	
					
					round_cnt.byte_data[3]=reviced_tem[i++]; 
					round_cnt.byte_data[2]=reviced_tem[i++];
					round_cnt.byte_data[1] = reviced_tem[i++] ; 
					round_cnt.byte_data[0] = reviced_tem[i++] ;
					moto_chassis[j].round_cnt =  round_cnt.int32_dat;
					
					total_angle.byte_data[3]=reviced_tem[i++]; 
					total_angle.byte_data[2]=reviced_tem[i++];
					total_angle.byte_data[1] = reviced_tem[i++] ; 
					total_angle.byte_data[0] = reviced_tem[i++] ;
					moto_chassis[j].angle =  total_angle.int32_dat;
				}
				
				// ROS_INFO_STREAM("recived mickv3 chassis motor data" ); 
				// for(j=0;j<4;j++)
				// {
				// 	// 打印四个电机的转速、转角、温度等信息
				// 	ROS_INFO_STREAM("M "<< j <<"\t rpm: "<<moto_chassis[j].speed_rpm
				// 							<<": \t round_cnt: "<<moto_chassis[j].round_cnt
				// 							<<"  angle: "<<moto_chassis[j].angle );
				// }
			}
			else if (reviced_tem[3+step] ==0x08 ) //mickv3 4个电机状态数据
			{
				;
			}
			else if (reviced_tem[3+step] ==0xA7 ) //mickv3 里程计
			{
				i=4+step;
				odom.int16_dat = 0;
				odom.int16_dat=0;odom.byte_data[1] = reviced_tem[i++] ; odom.byte_data[0] = reviced_tem[i++] ;
				mickv3_chassis.vx = odom.int16_dat/1000.0f;

				odom.int16_dat = 0;
				odom.int16_dat=0;odom.byte_data[1] = reviced_tem[i++] ; odom.byte_data[0] = reviced_tem[i++] ;
				mickv3_chassis.vy = odom.int16_dat/1000.0f;

				odom.int16_dat = 0;
				odom.int16_dat=0;odom.byte_data[1] = reviced_tem[i++] ; odom.byte_data[0] = reviced_tem[i++] ;
				mickv3_chassis.wz = odom.int16_dat/1000.0f;
				 
				mickv3_chassis.available = 0x01;
				//printf("odom: %f\t%f\t%f\n",mickv3_chassis.vx,mickv3_chassis.vy,mickv3_chassis.wz);
			}
			else if (reviced_tem[3+step] ==0xA0 ) // IMU 数据
			{
				i=4+step;
				
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ;imu.byte_data[0] = reviced_tem[i++] ;
				imu_chassis.ax = imu.int16_dat;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				imu_chassis.ay = imu.int16_dat;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				imu_chassis.az = imu.int16_dat;
				
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ;imu.byte_data[0] = reviced_tem[i++] ;
				imu_chassis.gx = imu.int16_dat;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				imu_chassis.gy = imu.int16_dat;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				imu_chassis.gz = imu.int16_dat;
				
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				imu_chassis.mx = imu.int16_dat;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				imu_chassis.my = imu.int16_dat;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				imu_chassis.mz = imu.int16_dat;
				
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				imu_chassis.pitch = imu.int16_dat/100.0f;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				imu_chassis.roll = imu.int16_dat/100.0f;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				imu_chassis.yaw = imu.int16_dat/100.0f;
				
				publish_imu(imu_chassis,imu_pub);
				//printf("recived imu  data" ); 
			}
			else if (reviced_tem[3 + step] == 0xA3)
			{
				i = 4 + step;

				rc.ch1 = reviced_tem[i++];
				rc.ch1 = (rc.ch1 << 8) + reviced_tem[i++];
				rc.ch2 = reviced_tem[i++];
				rc.ch2 = (rc.ch2 << 8) + reviced_tem[i++];
				rc.ch3 = reviced_tem[i++];
				rc.ch3 = (rc.ch3 << 8) + reviced_tem[i++];
				rc.ch4 = reviced_tem[i++];
				rc.ch4 = (rc.ch4 << 8) + reviced_tem[i++];
				rc.sw1 = reviced_tem[i++];
				rc.sw2 = reviced_tem[i++];
				rc.sw3 = reviced_tem[i++];
				rc.sw4 = reviced_tem[i++];
				rc.type = reviced_tem[i++];// 1 DJI-DBUS   2 SBUS 遥控器类型
				rc.status = reviced_tem[i++];
				rc.update = 0x01;
				if (rc.ch1 >= (RC_MIN-200) && rc.ch1 <=(RC_MAX+200))
				{
					rc.available = 0x01;
				}
				else
				{
					printf("rc.chx < RC_MIN || rc.chx > RC_MAX");
				}
				// if(show_message)
				// {
				// 	ROS_INFO_STREAM("RC_Remotes date  ch1: " << rc.ch1 << " ch2: " << rc.ch2
				// 					 << " ch3: " << rc.ch3 << " ch4: " << rc.ch4 << " sw1: " 
				// 					 << rc.sw1 << " sw2: " << rc.sw2<< " sw3: " 
				// 					 << rc.sw3 << " sw4: " << rc.sw4<< " type: " << rc.type);
				// }
				return true;
			}
			else if (reviced_tem[3+step] ==0xAC ) // IO状态
			{

			}
			else
			{
				printf("unrecognize frame 0x%x \n",reviced_tem[3 + step]);
			}
			//return  true;
		}
		else
		{
			printf("frame head is wrong" ); 
			return  false;	
		}
		step+=len; 
	}
	return  true;	         
}

/**
 * @function 利用里程计数据实现位置估计
 * 
 */
float s1=0,s2=0,s3=0,s4=0;
float s1_last=0,s2_last=0,s3_last=0,s4_last=0;
float position_x=0,position_y=0,position_w=0;
static int motor_init_flag = 0;

double last_time=0, curr_time =0;
// 差速底盘  速度计算    安普斯电机
// 仅仅只是前轮转向模式
void calculate_chassisDiffX4_position_for_odometry(rclcpp::Node::SharedPtr node,
						rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub,
								rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub)
{
    rclcpp::Clock clock;
	float position_x_delta,position_y_delta,position_w_delta,position_r_delta;
	float linear_x,linear_y,linear_w;
		
	if(motor_init_flag == 0 && ((s1_last == 0 && s2_last == 0&& s3_last==0&&s4_last==0) || (moto_chassis[0].counter ==0)))
	{
		position_x = 0 ; 
		position_y =0 ; 
		position_w =0 ; 
		curr_time = clock.now().seconds();

		last_time =  curr_time;
		motor_init_flag++;//保证程序只进入一次
		return ;
	}

	if(mickv3_chassis.available = 0x01) //直接使用底盘上传的里程计数据
	{
		linear_x = mickv3_chassis.vx;
		linear_y = 0;
		linear_w = mickv3_chassis.wz;  
	}

	//设置死区
	float linear_x_min = 0.01;// m/s
	float linear_w_min = 0.001;// rad/s 

	if(abs(linear_x)<linear_x_min)
	{
		linear_x=0;	
	}
	if(abs(linear_w)<linear_w_min)
	{
		linear_w=0;	
	}

	curr_time = clock.now().seconds();
	double dt = curr_time - last_time;
	if(dt>1)
		dt = 0;
	last_time =  curr_time;

	//ROS_INFO_STREAM(" calculate_chassisDiffX4_position_for_odometry dt:  "<<dt<<"  vx:  "<<linear_x<<"   vy: " <<linear_y<<"   vw: " <<linear_w);

   
	position_x=position_x+cos(position_w)*linear_x*dt;
	position_y=position_y+sin(position_w)*linear_x*dt;
	position_w=position_w+linear_w*dt;
 

	if(position_w>2*WHEEL_PI)
	{
		position_w=position_w-2*WHEEL_PI;	
	}
	else if(position_w<-2*WHEEL_PI)
	{
		position_w=position_w+2*WHEEL_PI;
	}
	else;

 	//ROS_INFO_STREAM("  position_x:  "<<position_x<<"  position_y:  "<<position_y<<"   position_w: " <<position_w); 
    publish_odomtery( node ,odom_pub,path_pub,position_x,position_y,position_w,linear_x,linear_y,linear_w);
    
}

/**
 * @function 发布里程计的数据
 * 
 */
void publish_odomtery(rclcpp::Node::SharedPtr node,rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub,
						rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub,
							float  position_x,float position_y,float oriention,
								float vel_linear_x,float vel_linear_y,float vel_linear_w)
{
	rclcpp::Clock clock;
	// static tf::TransformBroadcaster odom_broadcaster;  //定义tf对象
    static std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    geometry_msgs::msg::TransformStamped odom_trans;  //创建一个tf发布需要使用的TransformStamped类型消息
    geometry_msgs::msg::Quaternion odom_quat;   //四元数变量
    nav_msgs::msg::Odometry odom;  //定义里程计对象

		
	//里程计的偏航角需要转换成四元数才能发布
	odom_quat = tf2::toMsg(tf2::Quaternion(0, 0, std::sin(oriention / 2), std::cos(oriention / 2)));

	//载入坐标（tf）变换时间戳
	odom_trans.header.stamp = clock.now();
	//发布坐标变换的父子坐标系
	odom_trans.header.frame_id = "odom";     
	odom_trans.child_frame_id = "base_link";       
	//tf位置数据：x,y,z,方向
	odom_trans.transform.translation.x = position_x;
	odom_trans.transform.translation.y = position_y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;        
	//发布tf坐标变化
	odom_broadcaster->sendTransform(odom_trans);

	//载入里程计时间戳
	odom.header.stamp = clock.now();
	//里程计的父子坐标系
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";       
	//里程计位置数据：x,y,z,方向
	odom.pose.pose.position.x = position_x;     
	odom.pose.pose.position.y = position_y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;       
	//载入线速度和角速度
	odom.twist.twist.linear.x = vel_linear_x;
	odom.twist.twist.linear.y = vel_linear_y;
	odom.twist.twist.angular.z = vel_linear_w;    
	//发布里程计
	odom_pub->publish(odom);

	if(is_pub_path)
	{
		//发布小车里程计数据推算的轨迹
		geometry_msgs::msg::PoseStamped this_pose_stamped;
		this_pose_stamped.pose = odom.pose.pose;

		path.header.stamp=odom.header.stamp;
		path.header.frame_id="odom";
		path.poses.push_back(this_pose_stamped);
		path_pub->publish(path);
	}
}
void publish_imu(imu_measure_t& imu_m,rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub)
{
	rclcpp::Clock clock;
	sensor_msgs::msg::Imu IMU_msg;  //定义里程计对象
	//载入时间戳
	IMU_msg.header.stamp = clock.now();
	IMU_msg.header.frame_id = "imu";        

	Eigen::Vector3d ea0(imu_m.roll * M_PI / 180.0,
			imu_m.pitch * M_PI / 180.0,
			imu_m.yaw * M_PI / 180.0);
	Eigen::Matrix3d R;
	R = Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
		* Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());

	Eigen::Quaterniond q;
	q = R;
	q.normalize();

	IMU_msg.orientation.w = (double)q.w();
	IMU_msg.orientation.x = (double)q.x();
	IMU_msg.orientation.y = (double)q.y();
	IMU_msg.orientation.z = (double)q.z();

	IMU_msg.angular_velocity.x = imu_m.gx;   
	IMU_msg.angular_velocity.y = imu_m.gy;   
	IMU_msg.angular_velocity.z = imu_m.gz; 

	IMU_msg.linear_acceleration.x = imu_m.ax;   
	IMU_msg.linear_acceleration.y = imu_m.ay;   
	IMU_msg.linear_acceleration.z = imu_m.az; 

	// double accel = sqrt(imu_m.ax^2 + imu_m.ay^2 +imu_m.az^2 );//归一化
	// IMU_msg.linear_acceleration.x = imu_m.ax/accel*9.8;   
	// IMU_msg.linear_acceleration.y = imu_m.ay/accel*9.8;   
	// IMU_msg.linear_acceleration.z = imu_m.az/accel*9.8; 
 
	//发布IMU
	imu_pub->publish(IMU_msg);	 
}
// 发布遥控器数据
bool publish_joy_msg(rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub)
{
	if(rc.update != 0x01)
	{
		printf("rc.update != 0x01 ");
		return false;
	}

	// 0 未标定   1 标定中   2标定完成
	if(rc_init_flags != 2)
	{
		printf("rc.state != 0x02");
		int flag = calculate_rc_offset();
		//ROS_WARN_STREAM("flag "<<flag<<"  rc.state "<<rc.state);
		if(flag == 0)
		{
			rc_init_flags = 0 ;
			printf("calculate_rc_offset failed .... ");
		}
		else if(flag == 1)
		{
			rc_init_flags =1 ;
			printf("initial .... ");
		}
	}
	else
	{
		if (rc.available == 1) 
		{
			 
			float ch1,ch2,ch3,ch4;
			 
			float rc_k = 1;

			if (rc.sw2 == 1)            rc_k = 1;
			else if (rc.sw2 == 3)   rc_k = 2;
			else if (rc.sw2 == 2)  	rc_k = 3; // 3m/s
			else 				rc_k = 0;
			
			// 设置死区
			ch1 = (rc.ch1 - rc.ch1_offset) / (RC_MAX - rc.ch1_offset);
			if(abs(ch1)<0.2) ch1=0;
			ch2 =(rc.ch2 - rc.ch2_offset) / (RC_MAX - rc.ch2_offset);
			if(abs(ch2)<0.2) ch2=0;
			ch3 = (rc.ch3 - rc.ch3_offset) / (RC_MAX - rc.ch3_offset);
			if(abs(ch3)<0.2) ch3=0;
			ch4 =(rc.ch4 - rc.ch4_offset) / (RC_MAX - rc.ch4_offset);
			if(abs(ch4)<0.2) ch4=0;

			sensor_msgs::msg::Joy joy_msg;
			joy_msg.axes.push_back(RC_K *rc_k*ch1);
			joy_msg.axes.push_back(RC_K *rc_k*ch2);
			joy_msg.axes.push_back(RC_K *rc_k*ch3);
			joy_msg.axes.push_back(RC_K *rc_k*ch4);
			joy_msg.buttons.push_back(rc.sw1);
			joy_msg.buttons.push_back(rc.sw2);
			joy_pub->publish(joy_msg);
 
			return 1;
		}
	}
	rc.update = 0;
	 
	return 1;
}
// 计算遥控器的中位值
int calculate_rc_offset(void)
{
	int re_flag = 0;
	if(init_times < 20)
	{
		if ((rc.ch1 > 900 && rc.ch1 < 1100) && (rc.ch2 > 900 && rc.ch2 < 1100)
			&& (rc.ch3 > 900 && rc.ch3 < 1100) && (rc.ch4 > 900 && rc.ch4 < 1100))
		{
			sum_offset[0] += rc.ch1;
			sum_offset[1] += rc.ch2;
			sum_offset[2] += rc.ch3;
			sum_offset[3] += rc.ch4;
			rc.available = 0;
			rc_init_flags = 1; // 0 未标定   1 标定中   2标定完成
			init_times++;
			printf("calibrate...");
			re_flag = 1;
		}
	}
	else
	{
		rc.ch1_offset = sum_offset[0] / init_times;
		rc.ch2_offset = sum_offset[1] / init_times;
		rc.ch3_offset = sum_offset[2] / init_times;
		rc.ch4_offset = sum_offset[3] / init_times;
		cout<<"ch1_offset: " <<rc. ch1_offset << " ch2_offset: " << rc.ch2_offset
				 << "ch3_offset: " <<rc. ch3_offset << " ch4_offset: " << rc.ch4_offset<<endl;
		if (rc.ch1_offset == 0 || rc.ch2_offset == 0 || rc.ch3_offset == 0 || rc.ch4_offset == 0)
		{
			rc.available = 0;
			rc_init_flags = 0;
			init_times = 0;
			sum_offset[0] = 0;
			sum_offset[1] = 0;
			sum_offset[2] = 0;
			sum_offset[3] = 0;
			printf("calibrate faild...");
			re_flag = 0;
		}
		else
		{
			rc.available = 1;
			rc_init_flags = 0x02;
			printf("remote calibrate successful");
			re_flag = 2; //标定成功
		}
	}
	return re_flag; //标定中
}