extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <sys/time.h>
#include <mutex>

#include <cstdint>
#include <memory>
#include <string.h>
#include <cassert>
#include <string.h>
#include <thread>
#include <chrono>

#include "chassis_mick_msg.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>


#include "serial/serial.h"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

using namespace std;

int chassis_type = 0; //默认采用差速模式  0：差速  1-麦克纳姆轮  2: Ackermann  3:4WS4WD
int is_pub_path = 0; //默认不发布小车底盘轨迹  0：不发布   1 发布

float WHEEL_L=0.4;                 //左右轮子的间距
float WHEEL_D=0.17; 	   	//轮子直径  6.5寸的轮子
float WHEEL_R=WHEEL_D/2.0; 			//轮子半径
float WHEEL_PI=3.141693; 			//pi

serial::Serial ros_ser;

volatile rc_info_t rc;
int rc_init_flags =0;
unsigned int init_times = 0;
int sum_offset[4] = {0};
int show_message =1;
float RC_MIN = 0, RC_MAX = 2500, RC_K = 1; //遥控器摇杆通道输出的最小值、最大值、比例系数

chassis mickv3_chassis;

 
union INT32Data //union的作用为实现char数组和int32之间的转换
{
    int32_t int32_dat;
    unsigned char byte_data[4];
}motor_upload_counter,total_angle,round_cnt,speed_rpm;
union Int16Data //union的作用为实现char数组和int16数据类型之间的转换
{
    int16_t int16_dat;
    unsigned char byte_data[2];
}imu,odom;



// 创建一个空的 JSON 对象
json j_pose;
uint32_t count_1=0,count_2;
Geomsgs_Twist cmdvel_twist; // 
 
int run(void *dora_context);
void cmd_vel_callback(float x,float y,float w);
void send_speed_to_X4chassis(float x,float y,float w);
void send_rpm_to_chassis(int w1, int w2, int w3, int w4);
bool analy_uart_recive_data(string serial_data);
void calculate_chassisDiffX4_position_for_odometry(void *dora_context);



int main()
{
	std::cout << "Mickrobotx4 -v3 for dora " << std::endl;

  // 初始化串口
	string dev = "/dev/ttyUSB0";
	 
	int baud = 115200;
	int time_out = 1000;
	int hz = 100; 
	cout<<"dev:   "<<dev<<endl;
	cout<<"baud:   "<<baud<<endl;;
	cout<<"time_out:   "<<time_out<<endl;
	cout<<"hz:   "<<hz<<endl;
	cout<<"is_pub_path:   "<<chassis_type<<"\t  0:No  1:Yes"<<endl;
	cout<<"chassis_type:   "<<chassis_type<<"\t  0:X4  1:M4  2: Ackermann  3:4WS4WD"<<endl; 
 
	try
	{
		ros_ser.setPort(dev);
		ros_ser.setBaudrate(baud);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		//serial::Timeout to = serial::Timeout(1,time_out,0,time_out,0);
		to.inter_byte_timeout=1;
		to.read_timeout_constant=5;
		to.read_timeout_multiplier=0;
		ros_ser.setTimeout(to);
		ros_ser.open();
		ros_ser.flushInput(); //清空缓冲区数据
	}
	catch (serial::IOException& e)
	{
		cout<<"Unable to open port "<<endl;
		return -1;
	}
	if(ros_ser.isOpen())
	{
		ros_ser.flushInput(); //清空缓冲区数据
		cout<<"Serial Port opened"<<endl;
	}
	else
	{
		return -1;
	}

 	auto dora_context = init_dora_context_from_env();
	auto ret = run(dora_context);
	free_dora_context(dora_context);

	std::cout << "exit Mickrobotx4 node ..." << std::endl;
	return ret;
}

int run(void *dora_context)
{
    //std::mutex mtx_DoraNavSatFix;
    //std::mutex mtx_DoraQuaternionStamped; // mtx.unlock();
	bool uart_recive_flag;
	uint8_t chassis_type = 1;
	
    while(true)
    {
         
        void *event = dora_next_event(dora_context);
        
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            char *id;
            size_t id_len;
            read_dora_input_id(event, &id, &id_len);
			//cout<<"id_len: "<<id_len<<endl;

			if(ros_ser.available() )
			{
				//ROS_INFO_STREAM("Reading from serial port");
				std::string serial_data;
				//获取串口数据
				serial_data = ros_ser.read(ros_ser.available());
				//cout<<serial_data << "\n"<<endl;
				uart_recive_flag = analy_uart_recive_data(serial_data);
				if(uart_recive_flag)
				{
					uart_recive_flag=0;
					if(chassis_type == 0 || chassis_type == 1)
					{
						calculate_chassisDiffX4_position_for_odometry(dora_context);
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
					printf(" analy uart recive data error ...");
					//serial_data = ros_ser.read(ros_ser.available());
					ros_ser.flushInput(); //清空缓冲区数据
					//sleep(0.2);            //延时0.2秒,确保有数据进入
					std::this_thread::sleep_for(std::chrono::milliseconds(200));

				}
			}



            if (strncmp(id, "CmdVelTwist",11) == 0)
            {
				char *data;
				size_t data_len;
				read_dora_input_data(event, &data, &data_len);

				json j_cmd_vel;
				// 将数据转化为字符串
				std::string data_str(data, data_len);
				try 
				{
					j_cmd_vel = json::parse(data_str); // 解析 JSON 字符串               
				} 
				catch (const json::parse_error& e) 
				{
					std::cerr << "JSON 解析错误：" << e.what() << std::endl; // 处理解析失败的情况
					//free_dora_event(event);
				}
				
				count_1++;
				struct timeval tv;
				gettimeofday(&tv, NULL);

				cout << "Twist event count: "<<count_1<<" data_seq "<< j_cmd_vel["seq"]<<" time is: " << 
					 std::fixed << std::setprecision(9) << tv.tv_sec +tv.tv_usec*1e-9<<" s " <<std::endl;
				std::cout << "<----print---->" <<j_cmd_vel<< std::endl;
				cmdvel_twist.header.frame_id = j_cmd_vel["header"]["frame_id"];
				cmdvel_twist.header.seq = 	j_cmd_vel ["header"]["seq"];
				cmdvel_twist.header.sec = j_cmd_vel["header"]["stamp"]["sec"];
				cmdvel_twist.header.nanosec = j_cmd_vel["header"]["stamp"]["nanosec"];
				cmdvel_twist.linear.x = j_cmd_vel["linear"]["x"];
				cmdvel_twist.linear.y = j_cmd_vel["linear"]["y"];
				// cmdvel_twist.linear.z = j_cmd_vel["linear"]["z"];
				// cmdvel_twist.angular.x = j_cmd_vel["angular"]["x"];
				// cmdvel_twist.angular.y = j_cmd_vel["angular"]["y"];
				cmdvel_twist.angular.z = j_cmd_vel["angular"]["z"];
				 
				cmd_vel_callback(cmdvel_twist.linear.x,cmdvel_twist.linear.y,cmdvel_twist.angular.z);
			}
      }
      else if (ty == DoraEventType_Stop)
      {
          printf("[c node] received stop event\n");
      }
      else
      {
          printf("[c node] received unexpected event: %d\n", ty);
      }
      free_dora_event(event);

    }
    return 0;
}


void cmd_vel_callback(float speed_x,float speed_y,float speed_w)
{
  
	if(chassis_type == 0) //mick-v3 差速模式
	{
		//cout << "speed_x: "<<speed_x << "  speed_y: "<<speed_y<< "  speed_w: "<<speed_w<< endl;
		send_speed_to_X4chassis(speed_x,speed_y,speed_w);
		return ;
	}
	else
	{
		// RCLCPP_INFO_STREAM(node->get_logger(),"unknown chassis type ! ");
		cout << "unknown chassis type ! " << endl;
	}
 
}

/**
 * @function  发送四个点击的转速到底盘控制器
 * ＠param w1 w2 w3 w4 表示四个电机的转速 单位　RPM
 */
void send_rpm_to_chassis( int w1, int w2, int w3, int w4)
{
	uint8_t data_tem[50];
	unsigned int speed_0ffset=10000; //转速偏移10000转

	unsigned char i,counter=0;
	unsigned char  cmd;
	unsigned int check=0;
	cmd =0xF1;
	data_tem[counter++] =0xAE;
	data_tem[counter++] =0xEA;
	data_tem[counter++] =0x0B;
	data_tem[counter++] =cmd;

	data_tem[counter++] =(w1+speed_0ffset)/256; // 
	data_tem[counter++] =(w1+speed_0ffset)%256;

	data_tem[counter++] =(w2+speed_0ffset)/256; // 
	data_tem[counter++] =(w2+speed_0ffset)%256;

	data_tem[counter++] =(w3+speed_0ffset)/256; // 
	data_tem[counter++] =(w3+speed_0ffset)%256;

	data_tem[counter++] =(w4+speed_0ffset)/256; // 
	data_tem[counter++] =(w4+speed_0ffset)%256;

	for(i=2;i<counter;i++)
	{
	check+=data_tem[i];
	}
	data_tem[counter++] = check;
	data_tem[2] =counter-2;
	data_tem[counter++] =0xEF;
	data_tem[counter++] =0xFE;

	ros_ser.write(data_tem,counter);
}

// 差速小车
void send_speed_to_X4chassis(float x,float y,float w)
{
	uint8_t data_tem[50];
	unsigned int speed_0ffset=10; //速度偏移值 10ｍ/s，把速度转换成正数发送
	unsigned char i,counter=0;	 
	unsigned int check=0;

	data_tem[counter++] =0xAE;
	data_tem[counter++] =0xEA;
	data_tem[counter++] =0x0B;
	data_tem[counter++] = 0xF3; //针对MickX4的小车使用F3 字段      针对MickM4的小车使用F2
	data_tem[counter++] =((x+speed_0ffset)*100)/256; // X
	data_tem[counter++] =((x+speed_0ffset)*100);
	data_tem[counter++] =((y+speed_0ffset)*100)/256; // Y
	data_tem[counter++] =((y+speed_0ffset)*100);
	data_tem[counter++] =((w+speed_0ffset)*100)/256; // X
	data_tem[counter++] =((w+speed_0ffset)*100);
	data_tem[counter++] =0x00;
	data_tem[counter++] =0x00;
	for(i=2;i<counter;i++)
	{
		check+=data_tem[i];
	}
	data_tem[counter++] = check;
	data_tem[2] =counter-2;
	data_tem[counter++] =0xEF;
	data_tem[counter++] =0xFE;
	ros_ser.write(data_tem,counter);
}

void clear_odometry_chassis(void)
{
  uint8_t data_tem[50];
  unsigned char i,counter=0;
  unsigned char  cmd,resave=0x00;
  unsigned int check=0;
  cmd =0xE1;
  data_tem[counter++] =0xAE;
  data_tem[counter++] =0xEA;
  data_tem[counter++] =0x0B;
  data_tem[counter++] =cmd;
  
  data_tem[counter++] =0x01; //  清零里程计
  data_tem[counter++] =resave;
  
  data_tem[counter++] =resave; // 
  data_tem[counter++] =resave;
  
  data_tem[counter++] =resave; // 
  data_tem[counter++] =resave;
  
  data_tem[counter++] =resave; // 
  data_tem[counter++] =resave;
  
 
  for(i=2;i<counter;i++)
  {
    check+=data_tem[i];
  }
  data_tem[counter++] = check;
  data_tem[2] =counter-2;
  data_tem[counter++] =0xEF;
  data_tem[counter++] =0xFE;
 
 ros_ser.write(data_tem,counter);
  
}

/**
 * @function 解析串口发送过来的数据帧
 * 成功则返回true　否则返回false
 */
bool analy_uart_recive_data(string serial_data)
{
	unsigned char reviced_tem[500];
	uint16_t len=0,i=0,j=0;
	//unsigned char check=0;
	unsigned char tem_last=0,tem_curr=0,rec_flag=0;//定义接收标志位
	uint16_t header_count=0,step=0; //计数这个数据序列中有多少个帧头
	len=serial_data.size();
	if(len<1 || len>500)
	{
		cout<<"serial data is too short ,  len: " <<serial_data.size() <<endl;
		string str_tem = ros_ser.read(ros_ser.available());
		return false; //数据长度太短　
	}
	//ROS_INFO_STREAM("Read: " << serial_data.size() );

	// 有可能帧头不在第一个数组位置
	for( i=0;i<len;i++) 
	{
		tem_last=  tem_curr;
		tem_curr = serial_data.at(i);
		if(tem_last == 0xAE && tem_curr==0xEA&&rec_flag==0) //在接受的数据串中找到帧头　
		{
			rec_flag=1;
			reviced_tem[j++]=tem_last;
			reviced_tem[j++]=tem_curr;
			//ROS_INFO_STREAM("found frame head" ); 
		}
		else if (rec_flag==1)
		{
			reviced_tem[j++]=serial_data.at(i);
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
			if (reviced_tem[3+step] ==0xA7 ) //mickv3 里程计
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
 
float position_x=0,position_y=0,position_w=0;
static int motor_init_flag = 0;

double last_time=0, curr_time =0;
// 差速底盘  速度计算    安普斯电机
void calculate_chassisDiffX4_position_for_odometry(void *dora_context)
{
	//float position_x_delta,position_y_delta,position_w_delta,position_r_delta;
	float linear_x = 0,linear_y = 0,linear_w = 0;
		
	if(motor_init_flag == 0 )
	{
		position_x = 0 ; 
		position_y =0 ; 
		position_w =0 ; 
		//curr_time = clock.now().seconds();
		struct timeval tv;
		gettimeofday(&tv, NULL);
		curr_time = tv.tv_sec+tv.tv_usec*1e-9;
		cout << "first_time: "<<std::fixed << std::setprecision(9) << curr_time <<" s " <<std::endl;
		
		last_time =  curr_time;
		motor_init_flag++;//保证程序只进入一次
		return ;
	}

	if(mickv3_chassis.available == 0x01) //直接使用底盘上传的里程计数据
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

	struct timeval tv_curr;
	gettimeofday(&tv_curr, NULL);
	curr_time = tv_curr.tv_sec+tv_curr.tv_usec*1e-9;
	//cout << "curr_time: "<<std::fixed << std::setprecision(9) << curr_time <<" s " <<std::endl;
	//curr_time = clock.now().seconds();
	double dt = curr_time - last_time;
	if(dt>1)
		dt = 0;
	last_time =  curr_time;
	cout << "dt: "<<std::fixed << std::setprecision(9) << dt<<endl; 
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

 	//std::cout<<"  position_x:  "<<position_x<<"  position_y:  "<<position_y<<"   position_w: " <<position_w<<std::endl; 
    std::cout<<"  linear_x:  "<<linear_x<<"  position_y:  "<<linear_y<<"   linear_w: " <<linear_w<<std::endl; 
  //publish_odomtery( node ,odom_pub,path_pub,position_x,position_y,position_w,linear_x,linear_y,linear_w);
 
	json j_odom_pub;

	j_odom_pub["pose"]["position"]["x"] = position_x;
	j_odom_pub["pose"]["position"]["y"] = position_y;
	j_odom_pub["pose"]["position"]["z"] = 0;

	j_odom_pub["pose"]["orientation"]["x"] = 0;
	j_odom_pub["pose"]["orientation"]["y"] = 0;
	j_odom_pub["pose"]["orientation"]["z"] = 0;
	j_odom_pub["pose"]["orientation"]["w"] = 1;

	j_odom_pub["twist"]["linear"]["x"] = linear_x;
	j_odom_pub["twist"]["linear"]["y"] = linear_y;
	j_odom_pub["twist"]["linear"]["z"] = 0;

	j_odom_pub["twist"]["angular"]["x"] = 0;
	j_odom_pub["twist"]["angular"]["y"] = 0;
	j_odom_pub["twist"]["angular"]["z"] = linear_w;


	// 将 JSON 对象序列化为字符串
	std::string json_string = j_odom_pub.dump(4); // 参数 4 表示缩进宽度
	// 将字符串转换为 char* 类型
	char *c_json_string = new char[json_string.length() + 1];
	strcpy(c_json_string, json_string.c_str());
	std::string out_id = "Odometry";
	// std::cout<<json_string;
	int result = dora_send_output(dora_context, &out_id[0], out_id.length(), c_json_string, std::strlen(c_json_string));
	if (result != 0)
	{
		std::cerr << "failed to send output" << std::endl;
	}
	//std::cout << "dora_send_output()" << std::endl;
}