#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ==============linux driver===============
#include     <stdio.h>      /*标准输入输出的定义*/
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*UNIX 标准函数定义*/
#include     <sys/types.h>  /**/
#include     <sys/stat.h>  
#include     <fcntl.h>	    /*文件控制定义*/
#include     <termios.h>    /*PPSIX 终端控制定义*/
#include     <errno.h>      /*错误号定义*/
#include     <sys/time.h>
#include     <getopt.h>

// ==============ros driver================

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/accel.hpp"

#include "yesense_interface/msg/imu_data.hpp"
#include "yesense_interface/msg/imu_data_ten_axis.hpp"
#include "yesense_interface/msg/euler_only.hpp"
#include "yesense_interface/msg/robot_lord.hpp"
#include "yesense_interface/msg/attitude_min_vru.hpp"
#include "yesense_interface/msg/attitude_min_ahrs.hpp"
#include "yesense_interface/msg/attitude_all_data.hpp"
#include "yesense_interface/msg/pos_only.hpp"
#include "yesense_interface/msg/nav_min.hpp"
#include "yesense_interface/msg/nav_min_utc.hpp"
#include "yesense_interface/msg/nav_all.hpp"

#include "yesense_decoder_comm.h" 
#include "yesense_decoder.h"


#include <stdio.h>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <sys/file.h>
#include <thread>

// ===================================================================
#define CNT_PER_SECOND					1000u

#define UART_RX_BUF_LEN					512u
#define DEG_TO_RAD_FACTOR				57.29577952383886
static uint16_t PACKET_SIZE_input = 1206;
static uint16_t MSOP_DATA_PORT_NUMBER = 2368;   // lslidar default data port on PC
static uint16_t DIFOP_DATA_PORT_NUMBER = 2369;  // lslidar default difop data port on PC
#define MAX_ACK_BUF_LEN 4096

#define BAUDRATE_CNT					8

// ===================================================================
// ====== enumeration ======
typedef enum
{
	serial_drv_ros 		= 1,
	serial_drv_linux	= 2,
	serial_drv_unknown 	= -10
}serial_drv_t;

// ====== structure ======
typedef struct
{
	unsigned int flg;
	unsigned int timing_cnt;
	unsigned int msg_cnt;	
	unsigned int msg_rate;
}user_info_t;


// ===================================================================
using namespace std::chrono_literals;
using namespace yesense;

int g_baudrate_table[BAUDRATE_CNT] =
{
	9600, 19200, 38400, 57600, 115200,
	230400, 460800, 921600
};	// 目前仅支持这些波特率 

int g_baudrate_table_linux[BAUDRATE_CNT] =
{
	B9600, B19200, B38400, B57600, B115200,
	B230400, B460800, B921600
};	// 目前仅支持这些波特率 

// ===================================================================
speed_t obt_baudrate_from_int_to_linux(int baudrate)
{
	speed_t speed = B0;
	unsigned short i = 0u;

	for(i = 0u; i < BAUDRATE_CNT; i++)
	{
		if(baudrate == g_baudrate_table[i])
		{
			speed = g_baudrate_table_linux[i];
		}
	}

	return speed;
}

class YESENSE_Publisher : public rclcpp::Node
{
	public:
	YESENSE_Publisher()
	: Node("yesense_publisher")
	{

		this->declare_parameter<std::string>("frame_id", "basic_id");
		this->declare_parameter<std::string>("driver_type", "ros_serial");
		this->declare_parameter<std::string>("imu_topic_ros", "imu_data_ros");
		this->declare_parameter<std::string>("imu_topic", "imu_data");
		
		std::string driver_type_str,imu_topic_ros,imu_topic;
		this->get_parameter("frame_id", frame_id);
		this->get_parameter("driver_type", driver_type_str);	
		this->get_parameter("imu_topic_ros", imu_topic_ros);
		this->get_parameter("imu_topic", imu_topic);	

    	this->declare_parameter<std::string>("device_ip", std::string("192.168.1.200"));
    	this->declare_parameter<std::string>("device_ip_difop", std::string("192.168.1.102"));
    	this->declare_parameter<bool>("add_multicast", false);
    	this->declare_parameter<std::string>("group_ip", "224.1.1.2");
    	this->declare_parameter<int>("difop_port", 2369);
    	int snh_private_id = 2368 ;
    	this->declare_parameter<int>("device_port", 2368);
    	this->get_parameter("device_ip", devip_str_);
    	this->get_parameter("device_ip_difop", devip_str_difop);
    	this->get_parameter("add_multicast", add_multicast);
    	this->get_parameter("group_ip", group_ip);
    	this->get_parameter("difop_port", UDP_PORT_NUMBER_DIFOP);
    	this->get_parameter("device_port", device_port);


		RCLCPP_INFO(this->get_logger(), "frame id %s\n", frame_id.c_str());
		pub_imu_ros 		= this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_ros.c_str(), 10);				
		pub_pose_geometry 	= this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_geo", 10);		
		pub_marker 			= this->create_publisher<visualization_msgs::msg::Marker>("marker", 10);	

		// ==========================================		
		pub_imu 			= this->create_publisher<yesense_interface::msg::ImuData>(imu_topic.c_str(), 10);		
		pub_sensor_10axis	= this->create_publisher<yesense_interface::msg::ImuDataTenAxis>("sensor_10axis", 10);	
		pub_euler 			= this->create_publisher<yesense_interface::msg::EulerOnly>("euler_only", 10);
		pub_robt_lord		= this->create_publisher<yesense_interface::msg::RobotLord>("robot_lord", 10);
		pub_att_min_vru		= this->create_publisher<yesense_interface::msg::AttitudeMinVru>("att_min_vru", 10);
		pub_att_min_ahrs	= this->create_publisher<yesense_interface::msg::AttitudeMinAhrs>("att_min_ahrs", 10);		
		pub_att_all			= this->create_publisher<yesense_interface::msg::AttitudeAllData>("att_all", 10);
		pub_pos				= this->create_publisher<yesense_interface::msg::PosOnly>("pos_only", 10);
		pub_nav_min			= this->create_publisher<yesense_interface::msg::NavMin>("nav_min", 10);
		pub_nav_min_utc		= this->create_publisher<yesense_interface::msg::NavMinUtc>("nav_min_utc", 10);
		pub_nav_all			= this->create_publisher<yesense_interface::msg::NavAll>("nav_all", 10);

		
		// =================================================
		*((unsigned int *)&yis_out) = 0u;		
		user_info.flg 		= 1u;
		user_info.timing_cnt 	= 0u;
		user_info.msg_cnt		= 0u;
		user_info.msg_rate	= 0u;

		// =================================================
		initSerial();

		RCLCPP_INFO(this->get_logger(), "open serial port to decode msg!\n");			

		// =================================================
		timer_ 			= this->create_wall_timer(1ms, std::bind(&YESENSE_Publisher::timer_callback, this));
		timer_msg_rate_ = this->create_wall_timer(1ms, std::bind(&YESENSE_Publisher::callback_msg_rate_calc, this));		
	}

	~YESENSE_Publisher()
	{
		RCLCPP_INFO(this->get_logger(), "Close yesense device.");
    	(void)close(sockfd_);	
	}

	private:
	bool ReadFromIO(uint8_t *rx_buf, uint32_t rx_buf_len,uint32_t *rx_len);
    bool WriteToIo(const uint8_t *tx_buf, uint32_t tx_buf_len);
    void initSerial();

	void timer_callback()
	{
		uint32_t readed = 0;
	
		bool res = ReadFromIO(r_buffer, MAX_ACK_BUF_LEN, &readed);
                //ROS_INFO("Read data size: %d",data_.length());
        if (res&&readed)
        {
			int ret = decoder.data_proc(r_buffer, (unsigned int)readed, &yis_out);
			if(analysis_ok == ret)
			{
				if(yis_out.content.valid_flg)
				{
					yis_out.content.valid_flg = 0u;
					user_info.msg_cnt++;						
					// RCLCPP_INFO(this->get_logger(), "msg rate: %d, tid %d, acc: %f, %f, %f gyro: %f, %f, %f, euler: %f, %f, %f!", 
					// 			user_info.msg_rate,
					// 			yis_out.tid, yis_out.acc.x, yis_out.acc.y, yis_out.acc.z, yis_out.gyro.x, yis_out.gyro.y, yis_out.gyro.z,
					// 			yis_out.euler.pitch, yis_out.euler.roll, yis_out.euler.yaw
					// 		);
					publish_msg(&yis_out);
				}
			}
		}
	}
	
	void callback_msg_rate_calc()
	{
		if(user_info.flg)
		{
			user_info.timing_cnt++;
			if(user_info.timing_cnt >= CNT_PER_SECOND)
			{
				user_info.timing_cnt 	= 0u;
				user_info.msg_rate	= user_info.msg_cnt;
				user_info.msg_cnt		= 0u;
			}
		}
	}

	void publish_msg(yis_out_data_t *result);

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr timer_msg_rate_;	

	// serial
	unsigned char r_buffer[UART_RX_BUF_LEN];
	uint16_t port_;
  std::string devip_str_;
  int cur_rpm_;
  int return_mode_;
  bool npkt_update_flag_;
  bool add_multicast;
  std::string group_ip;
  int UDP_PORT_NUMBER_DIFOP;
  int socket_id_difop;
  int sockfd_;
  std::string devip_str_difop;
  std::thread *rx_thread_;
  long long rx_count_;
  int32_t com_handle_;
  uint32_t com_baudrate_;
  int device_port ;
  std::atomic<bool> is_cmd_opened_, rx_thread_exit_flag_;
  std::function<void(const char *, size_t length)> read_callback_;
  static void RxThreadProc(void *param);

  in_addr devip_;
  in_addr devip_difop;

	// configuration
	std::string serial_port;
	int baud_rate;
	std::string frame_id;	

	// ===
	yis_out_data_t yis_out;
	user_info_t user_info;

	// ros2 original topic
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_ros;		
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_geometry;	
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker;	

	// yesense self-defined topic
	rclcpp::Publisher<yesense_interface::msg::ImuData>::SharedPtr pub_imu;							// only inertia data: tid/acc/gyro/sensor_temp/sample_timestamp
	rclcpp::Publisher<yesense_interface::msg::ImuDataTenAxis>::SharedPtr pub_sensor_10axis;			// only inertia data: tid/acc/gyro/sensor_temp/sample_timestamp/mag/pressure(reserved)	
	rclcpp::Publisher<yesense_interface::msg::EulerOnly>::SharedPtr pub_euler;						// only euler: tid/pitch/roll/yaw
	rclcpp::Publisher<yesense_interface::msg::RobotLord>::SharedPtr pub_robt_lord;					// tid/acc/gyro/quatenion	
	rclcpp::Publisher<yesense_interface::msg::AttitudeMinVru>::SharedPtr pub_att_min_vru;			// minimum recommended data for attitude vru product: imu/euler	
	rclcpp::Publisher<yesense_interface::msg::AttitudeMinAhrs>::SharedPtr pub_att_min_ahrs;			// minimum recommended data for attitude ahrs product: imu/mag/euler		
	rclcpp::Publisher<yesense_interface::msg::AttitudeAllData>::SharedPtr pub_att_all;				// attitude all data: imu/mag/euler/quat
	rclcpp::Publisher<yesense_interface::msg::PosOnly>::SharedPtr pub_pos;							// only position: tid/latitude/longitude/altitude/nav_state	
	rclcpp::Publisher<yesense_interface::msg::NavMin>::SharedPtr pub_nav_min;						// recommended data for navigation product: tid/postion/euler/nav_state
	rclcpp::Publisher<yesense_interface::msg::NavMinUtc>::SharedPtr pub_nav_min_utc;				// recommended data for navigation product with utc: tid/postion/euler/utc/nav_state	
	rclcpp::Publisher<yesense_interface::msg::NavAll>::SharedPtr pub_nav_all;						// navigation all data: tid/acc/gyro/sensor_temp/eluer/quat/position/nav_state/vel/utc/pressure(reserved)

	yesense_decoder decoder;
};

void YESENSE_Publisher::publish_msg(yis_out_data_t *result)
{
	sensor_msgs::msg::Imu					imu_ros_data;
   	geometry_msgs::msg::PoseStamped 		pose_data;	
	visualization_msgs::msg::Marker			marker_info;

	yesense_interface::msg::ImuData 		imu_data;
	yesense_interface::msg::ImuDataTenAxis 	sensor_data_10axis;
	yesense_interface::msg::EulerOnly		euler_data;
	yesense_interface::msg::RobotLord		robot_data;
	yesense_interface::msg::AttitudeMinVru	att_min_vru_data;
	yesense_interface::msg::AttitudeMinAhrs	att_min_ahrs_data;
	yesense_interface::msg::AttitudeAllData	att_all_data;
	yesense_interface::msg::PosOnly			pos_data;
	yesense_interface::msg::NavMin			nav_min_data;
	yesense_interface::msg::NavMinUtc		nav_min_utc_data;
	yesense_interface::msg::NavAll			nav_all_data;

    // =========== publish imu message ==============
    imu_ros_data.header.stamp = this->get_clock()->now();

    imu_ros_data.orientation.x = result->quat.q1;
	imu_ros_data.orientation.y = result->quat.q2;
   	imu_ros_data.orientation.z = result->quat.q3;
   	imu_ros_data.orientation.w = result->quat.q0;      
 
    imu_ros_data.angular_velocity.x = result->gyro.x / DEG_TO_RAD_FACTOR;
    imu_ros_data.angular_velocity.y = result->gyro.y / DEG_TO_RAD_FACTOR;
    imu_ros_data.angular_velocity.z = result->gyro.z / DEG_TO_RAD_FACTOR;
    imu_ros_data.linear_acceleration.x = result->acc.x;
    imu_ros_data.linear_acceleration.y = result->acc.y;
    imu_ros_data.linear_acceleration.z = result->acc.z;
    pub_imu_ros->publish(imu_ros_data);

  	// ================ update pose =================
    pose_data.header.frame_id 	= frame_id;
    pose_data.header.stamp		= imu_ros_data.header.stamp;
    pose_data.pose.position.x 	= 0.0;
    pose_data.pose.position.y 	= 0.0;
    pose_data.pose.position.z 	= 0.0;
    pose_data.pose.orientation.w = imu_ros_data.orientation.w;
    pose_data.pose.orientation.x = imu_ros_data.orientation.x;
    pose_data.pose.orientation.y = imu_ros_data.orientation.y;
    pose_data.pose.orientation.z = imu_ros_data.orientation.z;
   	pub_pose_geometry->publish(pose_data);

    // ============ update imu marker ===============
    marker_info.header.frame_id = frame_id;
    marker_info.header.stamp 	= imu_ros_data.header.stamp;

    // set namespace and id
    marker_info.ns = "basic_shapes";
    marker_info.id = 0;

    // set marker's shape
    marker_info.type = visualization_msgs::msg::Marker::CUBE;

    // set action: ADD
    marker_info.action = visualization_msgs::msg::Marker::ADD;

    // set imu pose
    marker_info.pose.position.x 	= pose_data.pose.position.x;
    marker_info.pose.position.y 	= pose_data.pose.position.y;
    marker_info.pose.position.z 	= pose_data.pose.position.z;
    marker_info.pose.orientation.x 	= result->quat.q1;
    marker_info.pose.orientation.y 	= result->quat.q2;
    marker_info.pose.orientation.z 	= result->quat.q3;
    marker_info.pose.orientation.w 	= result->quat.q0;

    // set scale, unit: m
    marker_info.scale.x = 4.0;
    marker_info.scale.y = 4.0;
    marker_info.scale.z = 3.0;

    // set color
    marker_info.color.r = 0.2f;
    marker_info.color.g = 0.2f;
    marker_info.color.b = 0.2f;
    marker_info.color.a = 1.0;

	// foxy 版本需要使用该接口
	//marker_info.lifetime = rclcpp::Duration(10000000000u);

	// rolling 和 humble版本测试使用该函数
    marker_info.lifetime = rclcpp::Duration::from_nanoseconds(10000000000u);
    
    pub_marker->publish(marker_info);

	// ==============================================
	imu_data.tid.tid 	= result->tid;
	imu_data.acc.x 		= result->acc.x;
	imu_data.acc.y 		= result->acc.y;
	imu_data.acc.z 		= result->acc.z;
	imu_data.gyro.x		= result->gyro.x;
	imu_data.gyro.y		= result->gyro.y;
	imu_data.gyro.z		= result->gyro.z;
	imu_data.temp.temp	= result->sensor_temp;
	imu_data.sample_timestamp.timestamp = result->sample_timestamp;
	pub_imu->publish(imu_data);		

	sensor_data_10axis.imu_basic.tid.tid 	= result->tid;
	sensor_data_10axis.imu_basic.acc.x 		= result->acc.x;
	sensor_data_10axis.imu_basic.acc.y 		= result->acc.y;
	sensor_data_10axis.imu_basic.acc.z 		= result->acc.z;
	sensor_data_10axis.imu_basic.gyro.x		= result->gyro.x;
	sensor_data_10axis.imu_basic.gyro.y		= result->gyro.y;
	sensor_data_10axis.imu_basic.gyro.z		= result->gyro.z;
	sensor_data_10axis.imu_basic.temp.temp	= result->sensor_temp;
	sensor_data_10axis.mag_norm.x			= result->mag_norm.x;
	sensor_data_10axis.mag_norm.y			= result->mag_norm.y;
	sensor_data_10axis.mag_norm.z			= result->mag_norm.z;
	sensor_data_10axis.mag_raw.x			= result->mag_raw.x;
	sensor_data_10axis.mag_raw.y			= result->mag_raw.y;
	sensor_data_10axis.mag_raw.z			= result->mag_raw.z;
	sensor_data_10axis.imu_basic.sample_timestamp.timestamp = result->sample_timestamp;	
	pub_sensor_10axis->publish(sensor_data_10axis);

	euler_data.tid.tid 		= result->tid;
	euler_data.euler.pitch 	= result->euler.pitch;
	euler_data.euler.roll	= result->euler.roll;
	euler_data.euler.yaw 	= result->euler.yaw;
	pub_euler->publish(euler_data);

	robot_data.tid.tid 	= result->tid;
	robot_data.acc.x 	= result->acc.x;
	robot_data.acc.y 	= result->acc.y;
	robot_data.acc.z 	= result->acc.z;
	robot_data.gyro.x	= result->gyro.x;
	robot_data.gyro.y	= result->gyro.y;
	robot_data.gyro.z	= result->gyro.z;	
	robot_data.quat.q0	= result->quat.q0;
	robot_data.quat.q1	= result->quat.q1;
	robot_data.quat.q2	= result->quat.q2;
	robot_data.quat.q3	= result->quat.q3;			
	pub_robt_lord->publish(robot_data);

	att_min_vru_data.imu_basic.tid.tid	= result->tid;
	att_min_vru_data.imu_basic.acc.x 	= result->acc.x;
	att_min_vru_data.imu_basic.acc.y 	= result->acc.y;
	att_min_vru_data.imu_basic.acc.z 	= result->acc.z;
	att_min_vru_data.imu_basic.gyro.x	= result->gyro.x;
	att_min_vru_data.imu_basic.gyro.y	= result->gyro.y;
	att_min_vru_data.imu_basic.gyro.z	= result->gyro.z;
	att_min_vru_data.euler.pitch 		= result->euler.pitch;
	att_min_vru_data.euler.roll			= result->euler.roll;
	att_min_vru_data.euler.yaw 			= result->euler.yaw;	
	att_min_vru_data.imu_basic.temp.temp= result->sensor_temp;
	att_min_vru_data.imu_basic.sample_timestamp.timestamp = result->sample_timestamp;	
	pub_att_min_vru->publish(att_min_vru_data);

	att_min_ahrs_data.att_min_vru.imu_basic.tid.tid = result->tid;
	att_min_ahrs_data.att_min_vru.imu_basic.acc.x 	= result->acc.x;
	att_min_ahrs_data.att_min_vru.imu_basic.acc.y 	= result->acc.y;
	att_min_ahrs_data.att_min_vru.imu_basic.acc.z 	= result->acc.z;
	att_min_ahrs_data.att_min_vru.imu_basic.gyro.x	= result->gyro.x;
	att_min_ahrs_data.att_min_vru.imu_basic.gyro.y	= result->gyro.y;
	att_min_ahrs_data.att_min_vru.imu_basic.gyro.z	= result->gyro.z;
	att_min_ahrs_data.mag_norm.x					= result->mag_norm.x;
	att_min_ahrs_data.mag_norm.y					= result->mag_norm.y;
	att_min_ahrs_data.mag_norm.z					= result->mag_norm.z;
	att_min_ahrs_data.mag_raw.x						= result->mag_raw.x;
	att_min_ahrs_data.mag_raw.y						= result->mag_raw.y;
	att_min_ahrs_data.mag_raw.z						= result->mag_raw.z;	
	att_min_ahrs_data.att_min_vru.euler.pitch 		= result->euler.pitch;
	att_min_ahrs_data.att_min_vru.euler.roll		= result->euler.roll;
	att_min_ahrs_data.att_min_vru.euler.yaw 		= result->euler.yaw;	
	att_min_ahrs_data.att_min_vru.imu_basic.temp.temp					= result->sensor_temp;
	att_min_ahrs_data.att_min_vru.imu_basic.sample_timestamp.timestamp 	= result->sample_timestamp;		
	pub_att_min_ahrs->publish(att_min_ahrs_data);

	att_all_data.quat.q0 = result->quat.q0;
	att_all_data.quat.q1 = result->quat.q1;
	att_all_data.quat.q2 = result->quat.q2;
	att_all_data.quat.q3 = result->quat.q3;
	att_all_data.att_min_ahrs.att_min_vru.imu_basic.tid.tid = result->tid;
	att_all_data.att_min_ahrs.att_min_vru.imu_basic.acc.x 	= result->acc.x;
	att_all_data.att_min_ahrs.att_min_vru.imu_basic.acc.y 	= result->acc.y;
	att_all_data.att_min_ahrs.att_min_vru.imu_basic.acc.z 	= result->acc.z;
	att_all_data.att_min_ahrs.att_min_vru.imu_basic.gyro.x	= result->gyro.x;
	att_all_data.att_min_ahrs.att_min_vru.imu_basic.gyro.y	= result->gyro.y;
	att_all_data.att_min_ahrs.att_min_vru.imu_basic.gyro.z	= result->gyro.z;
	att_all_data.att_min_ahrs.mag_norm.x					= result->mag_norm.x;
	att_all_data.att_min_ahrs.mag_norm.y					= result->mag_norm.y;
	att_all_data.att_min_ahrs.mag_norm.z					= result->mag_norm.z;
	att_all_data.att_min_ahrs.mag_raw.x						= result->mag_raw.x;
	att_all_data.att_min_ahrs.mag_raw.y						= result->mag_raw.y;
	att_all_data.att_min_ahrs.mag_raw.z						= result->mag_raw.z;	
	att_all_data.att_min_ahrs.att_min_vru.euler.pitch 		= result->euler.pitch;
	att_all_data.att_min_ahrs.att_min_vru.euler.roll		= result->euler.roll;
	att_all_data.att_min_ahrs.att_min_vru.euler.yaw 		= result->euler.yaw;	
	att_all_data.att_min_ahrs.att_min_vru.imu_basic.temp.temp					= result->sensor_temp;
	att_all_data.att_min_ahrs.att_min_vru.imu_basic.sample_timestamp.timestamp 	= result->sample_timestamp;			
	pub_att_all->publish(att_all_data);

	pos_data.tid.tid 		= result->tid;
	pos_data.pos.longitude 	= result->pos.longitude;
	pos_data.pos.latitude	= result->pos.latitude;
	pos_data.pos.altitude	= result->pos.altitude;
	pos_data.status.fusion_status	= result->status.status.bit.fusion_sta;
	pos_data.status.gnss_status		= result->status.status.bit.pos_sta;
	pub_pos->publish(pos_data);

	nav_min_data.pos.tid.tid 		= result->tid;
	nav_min_data.pos.pos.longitude 	= result->pos.longitude;
	nav_min_data.pos.pos.latitude	= result->pos.latitude;
	nav_min_data.pos.pos.altitude	= result->pos.altitude;	
	nav_min_data.euler.pitch		= result->euler.pitch;
	nav_min_data.euler.roll			= result->euler.roll;
	nav_min_data.euler.yaw			= result->euler.yaw;
	nav_min_data.pos.status.fusion_status 	= result->status.status.bit.fusion_sta;
	nav_min_data.pos.status.gnss_status		= result->status.status.bit.pos_sta;
	pub_nav_min->publish(nav_min_data);

	nav_min_utc_data.nav_basic.pos.tid.tid 			= result->tid;
	nav_min_utc_data.nav_basic.pos.pos.longitude 	= result->pos.longitude;
	nav_min_utc_data.nav_basic.pos.pos.latitude		= result->pos.latitude;
	nav_min_utc_data.nav_basic.pos.pos.altitude		= result->pos.altitude;	
	nav_min_utc_data.nav_basic.euler.pitch			= result->euler.pitch;
	nav_min_utc_data.nav_basic.euler.roll			= result->euler.roll;
	nav_min_utc_data.nav_basic.euler.yaw			= result->euler.yaw;
	nav_min_utc_data.nav_basic.pos.status.fusion_status = result->status.status.bit.fusion_sta;
	nav_min_utc_data.nav_basic.pos.status.gnss_status	= result->status.status.bit.pos_sta;	
	nav_min_utc_data.utc.year	= result->utc.year;
	nav_min_utc_data.utc.month	= result->utc.month;
	nav_min_utc_data.utc.day	= result->utc.day;
	nav_min_utc_data.utc.hour	= result->utc.hour;
	nav_min_utc_data.utc.min	= result->utc.min;
	nav_min_utc_data.utc.sec	= result->utc.sec;
	nav_min_utc_data.utc.ms		= result->utc.ms;
	pub_nav_min_utc->publish(nav_min_utc_data);

	nav_all_data.tid.tid 		= result->tid;
	nav_all_data.acc.x 			= result->acc.x;
	nav_all_data.acc.y 			= result->acc.y;
	nav_all_data.acc.z 			= result->acc.z;
	nav_all_data.gyro.x			= result->gyro.x;
	nav_all_data.gyro.y			= result->gyro.y;
	nav_all_data.gyro.z			= result->gyro.z;
	nav_all_data.euler.pitch 	= result->euler.pitch;
	nav_all_data.euler.roll		= result->euler.roll;
	nav_all_data.euler.yaw 		= result->euler.yaw;	
	nav_all_data.quat.q0		= result->quat.q0;
	nav_all_data.quat.q1		= result->quat.q1;
	nav_all_data.quat.q2		= result->quat.q2;
	nav_all_data.quat.q3		= result->quat.q3;			
	nav_all_data.temp.temp		= result->sensor_temp;

	nav_all_data.pos.longitude 	= result->pos.longitude;
	nav_all_data.pos.latitude	= result->pos.latitude;
	nav_all_data.pos.altitude	= result->pos.altitude;	
	nav_all_data.status.fusion_status 	= result->status.status.bit.fusion_sta;
	nav_all_data.status.gnss_status		= result->status.status.bit.pos_sta;	
	nav_all_data.vel.vel_e		= result->vel.vel_e;
	nav_all_data.vel.vel_n		= result->vel.vel_n;
	nav_all_data.vel.vel_u		= result->vel.vel_u;
	nav_all_data.utc.year		= result->utc.year;
	nav_all_data.utc.month		= result->utc.month;
	nav_all_data.utc.day		= result->utc.day;
	nav_all_data.utc.hour		= result->utc.hour;
	nav_all_data.utc.min		= result->utc.min;
	nav_all_data.utc.sec		= result->utc.sec;
	nav_all_data.utc.ms			= result->utc.ms;	
	nav_all_data.pressure.val 	= result->pressure;
	pub_nav_all->publish(nav_all_data);
}

bool YESENSE_Publisher::ReadFromIO(uint8_t *rx_buf, uint32_t rx_buf_len,
                                   uint32_t *rx_len) {
        //double time1 = this->get_clock()->now();
        int q = 0;
        struct pollfd fds[1];
        fds[0].fd = sockfd_;
        fds[0].events = POLLIN;
        static const int POLL_TIMEOUT = 2000; // one second (in msec)

        sockaddr_in sender_address;
        socklen_t sender_address_len = sizeof(sender_address);
        // while (true)
        while (1)
        {
            // poll() until input available
            do
            {
                int retval = poll(fds, 1, POLL_TIMEOUT);
                if (retval < 0) // poll() error?
                {
                    if (errno != EINTR)
                        RCLCPP_INFO(this->get_logger(), "poll() error: %s", strerror(errno));
                    return 0;
                }
                if (retval == 0) // poll() timeout?
                {
                    RCLCPP_WARN(this->get_logger(), "lslidar poll() timeout");
                    return 0;
                }
                if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL)) // device error?
                {
                    RCLCPP_ERROR(this->get_logger(), "poll() reports lslidar error");
                    return 0;
                }
            } while ((fds[0].revents & POLLIN) == 0);

            // Receive packets that should now be available from the
            // socket using a blocking read.
            ssize_t nbytes = recvfrom(sockfd_, rx_buf, rx_buf_len, 0,
                                      (sockaddr *)&sender_address, &sender_address_len);
            //        ROS_DEBUG_STREAM("incomplete lslidar packet read: "
            //                         << nbytes << " bytes");
            q = (int)nbytes;
            if (nbytes < 0)
            {
                if (errno != EWOULDBLOCK)
                {
                    perror("recvfail");
                    RCLCPP_INFO(this->get_logger(), "recvfail");
                    return 1;
                }
            }
            else if ((size_t)nbytes <= rx_buf_len || (size_t)nbytes >= 50)
            {

                // read successful,
                // if packet is not from the lidar scanner we selected by IP,
                // continue otherwise we are done
                if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr)
                    continue;
                else
                    break; // done
            }
            
        }
        // if (flag == 0)
        // {
        //     abort();
        // }
        // this->getFPGA_GPSTimeStamp(packet);

        // Average the times at which we begin and end reading.  Use that to
        // estimate when the scan occurred.
        //double time2 = this->get_clock()->now().toSec();
        // packet->stamp = ros::Time((time2 + time1) / 2.0);
        // packet->stamp = this->timeStamp;
        //for(int i=0;i<rx_buf_len;i++)printHexData(rx_buf[i]);
        *rx_len = q;
        return q;
}

void YESENSE_Publisher::initSerial()
{
        sockfd_ = -1;
        if (!devip_str_.empty())
        {
            inet_aton(devip_str_.c_str(), &devip_);
            inet_aton(devip_str_difop.c_str(), &devip_difop);
        }
        RCLCPP_INFO_STREAM(this->get_logger(),"Opening UDP socket: port " << device_port);
        sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
        if (sockfd_ == -1)
        {
            printf("socket"); // TODO: ROS_ERROR errno
            return;
        }
        int opt = 1;
        if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void *)&opt, sizeof(opt)))
        {
            printf("setsockopt error!\n");
            return;
        }

        sockaddr_in my_addr;                         // my address information
        memset(&my_addr, 0, sizeof(my_addr));        // initialize to zeros
        my_addr.sin_family = AF_INET;                // host byte order
        my_addr.sin_port = htons(device_port);              // port in network byte order
        my_addr.sin_addr.s_addr = htonl(INADDR_ANY); // automatically fill in my IP

        if (bind(sockfd_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1)
        {
            printf("bind"); // TODO: ROS_ERROR errno
            return;
        }
        if (add_multicast)
        {
            struct ip_mreq group;
            // char *group_ip_ = (char *) group_ip.c_str();
            group.imr_multiaddr.s_addr = inet_addr(group_ip.c_str());
            // group.imr_interface.s_addr =  INADDR_ANY;
            group.imr_interface.s_addr = htonl(INADDR_ANY);
            // group.imr_interface.s_addr = inet_addr("192.168.1.102");

            if (setsockopt(sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&group, sizeof(group)) < 0)
            {
                printf("Adding multicast group error ");
                close(sockfd_);
                exit(1);
            }
            else
                printf("Adding multicast group...OK.\n");
        }

        if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
        {
            printf("non-block");
            return;
        }
}

bool YESENSE_Publisher::WriteToIo(const uint8_t *tx_buf, uint32_t tx_buf_len) 
{
        sockaddr_in server_sai;
        server_sai.sin_family = AF_INET; // IPV4 协议族
        server_sai.sin_port = htons(UDP_PORT_NUMBER_DIFOP);
        server_sai.sin_addr.s_addr = inet_addr(devip_str_.c_str());
        int rtn = 0;
        rtn = sendto(sockfd_, tx_buf, tx_buf_len, 0, (struct sockaddr *)&server_sai, sizeof(struct sockaddr));
        if (rtn < 0)
        {
            printf("start error !\n");
        }
        else
        {
            usleep(3000000);
            return 1;
        }
        return 0;
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);  
  
	rclcpp::spin(std::make_shared<YESENSE_Publisher>());
	rclcpp::shutdown();
	
		
	return 0;
}
