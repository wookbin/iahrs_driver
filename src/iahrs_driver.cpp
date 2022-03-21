#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>

//Service_include...
#include "iahrs_driver/all_data_reset.h"
#include "iahrs_driver/euler_angle_init.h"
#include "iahrs_driver/euler_angle_reset.h"
#include "iahrs_driver/pose_velocity_reset.h"
#include "iahrs_driver/reboot_sensor.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/fcntl.h>
#include <time.h>
#include <sys/types.h>
#include <vector>
#include <iostream>
#include <dirent.h>
#include <signal.h>

#define SERIAL_PORT		"/dev/IMU"
#define SERIAL_SPEED		B115200

typedef struct IMU_DATA
{
	double dQuaternion_x = 0.0;
	double dQuaternion_y = 0.0;
	double dQuaternion_z = 0.0;
	double dQuaternion_w = 1.0;

	double dAngular_velocity_x = 0.0;
	double dAngular_velocity_y = 0.0;
	double dAngular_velocity_z = 0.0;
	
	double dLinear_acceleration_x = 0.0;
	double dLinear_acceleration_y = 0.0;
	double dLinear_acceleration_z = 0.0;
    
	double dEuler_angle_Roll = 0.0;
	double dEuler_angle_Pitch = 0.0;
	double dEuler_angle_Yaw = 0.0;

}IMU_DATA;
IMU_DATA _pIMU_data;

int serial_fd = -1;
double time_offset_in_seconds;
double dSend_Data[10];
double m_dRoll, m_dPitch, m_dYaw;
sensor_msgs::Imu imu_data_msg;
//tf_prefix add
std::string tf_prefix_;
//single_used TF
bool m_bSingle_TF_option = false;

//IMU_service
ros::ServiceServer all_data_reset_service;
iahrs_driver::all_data_reset all_data_reset_cmd;
ros::ServiceServer euler_angle_init_service;
iahrs_driver::euler_angle_init euler_angle_init_cmd;
ros::ServiceServer euler_angle_reset_service;
iahrs_driver::euler_angle_reset euler_angle_reset_cmd;
ros::ServiceServer pose_velocity_reset_service;
iahrs_driver::pose_velocity_reset pose_velocity_reset_cmd;
ros::ServiceServer reboot_sensor_service;
iahrs_driver::reboot_sensor reboot_sensor_cmd;

int serial_open ()
{
	printf ("Try to open serial: %s\n", SERIAL_PORT); 

	serial_fd = open(SERIAL_PORT, O_RDWR|O_NOCTTY);
	if (serial_fd < 0) {
		printf ("Error unable to open %s\n", SERIAL_PORT);
		return -1;
	}
  	printf ("%s open success\n", SERIAL_PORT);

  	struct termios tio;
  	tcgetattr(serial_fd, &tio);
  	cfmakeraw(&tio);
	tio.c_cflag = CS8|CLOCAL|CREAD;
  	tio.c_iflag &= ~(IXON | IXOFF);
  	cfsetspeed(&tio, SERIAL_SPEED);
  	tio.c_cc[VTIME] = 0;
  	tio.c_cc[VMIN] = 0;

  	int err = tcsetattr(serial_fd, TCSAFLUSH, &tio);
  	if (err != 0) 
	{
    	printf ("Error tcsetattr() function return error\n");
    	close(serial_fd);
		serial_fd = -1;
    	return -1;
  	}
	return 0;
}

static unsigned long GetTickCount() 
{
    struct timespec ts;
   
    clock_gettime (CLOCK_MONOTONIC, &ts);

    return ts.tv_sec*1000 + ts.tv_nsec/1000000;
}

int SendRecv(const char* command, double* returned_data, int data_length)
{
	#define COMM_RECV_TIMEOUT	30	

	char temp_buff[256];
	read (serial_fd, temp_buff, 256);

	int command_len = strlen(command);
	int n = write(serial_fd, command, command_len);

	if (n < 0) return -1;

	const int buff_size = 1024;
	int  recv_len = 0;
	char recv_buff[buff_size + 1];

	unsigned long time_start = GetTickCount();

	while (recv_len < buff_size) {
		int n = read (serial_fd, recv_buff + recv_len, buff_size - recv_len);
		if (n < 0) 
		{
			return -1;
		}
		else if (n == 0) 
		{
			usleep(1000);
		}
		else if (n > 0) 
		{
			recv_len += n;

			if (recv_buff[recv_len - 1] == '\r' || recv_buff[recv_len - 1] == '\n') 
			{
				break;
			}
		}

		unsigned long time_current = GetTickCount();
		unsigned long time_delta = time_current - time_start;

		if (time_delta >= COMM_RECV_TIMEOUT) break;
	}
	recv_buff[recv_len] = '\0';

	if (recv_len > 0) 
	{
		if (recv_buff[0] == '!') 
		{
			return -1;
		}
	}

	if (strncmp(command, recv_buff, command_len - 1) == 0) {
		if (recv_buff[command_len - 1] == '=') {
			int data_count = 0;

			char* p = &recv_buff[command_len];
			char* pp = NULL;

			for (int i = 0; i < data_length; i++) 
			{
				if (p[0] == '0' && p[1] == 'x') 
				{
					returned_data[i] = strtol(p+2, &pp, 16);
					data_count++;
				}
				else 
				{
					returned_data[i] = strtod(p, &pp);
					data_count++;
				}

				if (*pp == ',') 
				{
					p = pp + 1;
				}
				else 
				{
					break;
				}
			}
			return data_count;
		}
	}
	return 0;
}

void my_handler(sig_atomic_t s)
{
    printf("Caught signal %d\n",s);
    exit(1); 
}

//ROS Service Callback////////////////////////////////////////////////////////////////////////
bool All_Data_Reset_Command(iahrs_driver::all_data_reset::Request  &req, 
					    	iahrs_driver::all_data_reset::Response &res)
{
	bool bResult = false;

    double dSend_Data[10];
	SendRecv("rc\n", dSend_Data, 10);

    bResult = true;
	res.command_Result = bResult;
	return true;
}

bool Euler_Angle_Init_Command(iahrs_driver::euler_angle_init::Request  &req, 
					    	  iahrs_driver::euler_angle_init::Response &res)
{
	bool bResult = false;

    double dSend_Data[10];
	SendRecv("za\n", dSend_Data, 10);

    bResult = true;
	res.command_Result = bResult;
	return true;
}

bool Euler_Angle_Reset_Command(iahrs_driver::euler_angle_reset::Request  &req, 
					    	   iahrs_driver::euler_angle_reset::Response &res)
{
	bool bResult = false;

    double dSend_Data[10];
	SendRecv("ra\n", dSend_Data, 10);

    bResult = true;
	res.command_Result = bResult;
	return true;
}

bool Pose_Velocity_Reset_Command(iahrs_driver::pose_velocity_reset::Request  &req, 
					    	     iahrs_driver::pose_velocity_reset::Response &res)
{
	bool bResult = false;

    double dSend_Data[10];
	SendRecv("rp\n", dSend_Data, 10);

    bResult = true;
	res.command_Result = bResult;
	return true;
}

bool Reboot_Sensor_Command(iahrs_driver::reboot_sensor::Request  &req, 
					       iahrs_driver::reboot_sensor::Response &res)
{
	bool bResult = false;

    double dSend_Data[10];
	SendRecv("rd\n", dSend_Data, 10);

    bResult = true;
	res.command_Result = bResult;
	return true;
}


int main (int argc, char** argv)
{
	signal(SIGINT,my_handler);
    	ros::init(argc, argv, "iahrs_driver");

	ros::NodeHandle private_node_handle("~");
    	private_node_handle.param<double>("time_offset_in_seconds", time_offset_in_seconds, 0.0);
	
	ros::param::get("tf_prefix", tf_prefix_);

	tf::TransformBroadcaster br;
  	tf::Transform transform;


	// These values do not need to be converted
	imu_data_msg.linear_acceleration_covariance[0] = 0.0064;
	imu_data_msg.linear_acceleration_covariance[4] = 0.0063;
	imu_data_msg.linear_acceleration_covariance[8] = 0.0064;

	imu_data_msg.angular_velocity_covariance[0] = 0.032*(M_PI/180.0);
	imu_data_msg.angular_velocity_covariance[4] = 0.028*(M_PI/180.0);
	imu_data_msg.angular_velocity_covariance[8] = 0.006*(M_PI/180.0);

	imu_data_msg.orientation_covariance[0] = 0.013*(M_PI/180.0);
	imu_data_msg.orientation_covariance[4] = 0.011*(M_PI/180.0);
	imu_data_msg.orientation_covariance[8] = 0.006*(M_PI/180.0);

    	ros::NodeHandle nh;
	ros::Publisher imu_data_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 1);
	
	//IMU Service///////////////////////////////////////////////////////////////////////////////////////////////
    	ros::NodeHandle sh;
    	all_data_reset_service = sh.advertiseService("all_data_reset_cmd", All_Data_Reset_Command);
	euler_angle_init_service = sh.advertiseService("euler_angle_init_cmd", Euler_Angle_Init_Command);
	euler_angle_reset_service = sh.advertiseService("euler_angle_reset_cmd", Euler_Angle_Reset_Command);
	pose_velocity_reset_service = sh.advertiseService("pose_velocity_reset_cmd", Pose_Velocity_Reset_Command);
	reboot_sensor_service = sh.advertiseService("reboot_sensor_cmd", Reboot_Sensor_Command);
	
	nh.getParam("m_bSingle_TF_option", m_bSingle_TF_option);
    	printf("##m_bSingle_TF_option: %d \n", m_bSingle_TF_option);

    	ros::Rate loop_rate(100); //HZ
    	serial_open();

	SendRecv("za\n", dSend_Data, 10);	// Euler Angle -> '0.0' Reset
	usleep(10000);

    while(ros::ok())
    {
        ros::spinOnce();

		if (serial_fd >= 0) 
		{
			const int max_data = 10;
			double data[max_data];
			int no_data = 0;

			no_data = SendRecv("g\n", data, max_data);	// Read angular_velocity _ wx, wy, wz 
			if (no_data >= 3) 
			{
				// angular_velocity
				imu_data_msg.angular_velocity.x = _pIMU_data.dAngular_velocity_x = data[0]*(M_PI/180.0);
				imu_data_msg.angular_velocity.y = _pIMU_data.dAngular_velocity_y = data[1]*(M_PI/180.0);
				imu_data_msg.angular_velocity.z = _pIMU_data.dAngular_velocity_z = data[2]*(M_PI/180.0);
				
			}

			no_data = SendRecv("a\n", data, max_data);	// Read linear_acceleration 	unit: m/s^2
			if (no_data >= 3) 
			{
				//// linear_acceleration   g to m/s^2
				imu_data_msg.linear_acceleration.x = _pIMU_data.dLinear_acceleration_x = data[0] * 9.80665;
				imu_data_msg.linear_acceleration.y = _pIMU_data.dLinear_acceleration_y = data[1] * 9.80665;
				imu_data_msg.linear_acceleration.z = _pIMU_data.dLinear_acceleration_z = data[2] * 9.80665;
			}

			no_data = SendRecv("e\n", data, max_data);	// Read Euler angle
			if (no_data >= 3) 
			{
				// Euler _ rad
				_pIMU_data.dEuler_angle_Roll  = data[0]*(M_PI/180.0);
				_pIMU_data.dEuler_angle_Pitch = data[1]*(M_PI/180.0);
				_pIMU_data.dEuler_angle_Yaw	  = data[2]*(M_PI/180.0);
			}

			//editing_
			tf::Quaternion orientation = tf::createQuaternionFromRPY(_pIMU_data.dEuler_angle_Roll , _pIMU_data.dEuler_angle_Pitch, _pIMU_data.dEuler_angle_Yaw);
			// orientation
			imu_data_msg.orientation.x = orientation[0];
			imu_data_msg.orientation.y = orientation[1];
			imu_data_msg.orientation.z = orientation[2];
			imu_data_msg.orientation.w = orientation[3];

			// calculate measurement time
            		ros::Time measurement_time = ros::Time::now() + ros::Duration(time_offset_in_seconds);

			imu_data_msg.header.stamp = measurement_time;
			imu_data_msg.header.frame_id = tf_prefix_ + "/imu_link";  // "imu_link"

			// publish the IMU data
			imu_data_pub.publish(imu_data_msg);

			//Publish tf
			if(m_bSingle_TF_option)
			{
				transform.setOrigin( tf::Vector3(0.0, 0.0, 0.2) );
				tf::Quaternion q;
				q.setRPY(_pIMU_data.dEuler_angle_Roll, _pIMU_data.dEuler_angle_Pitch, _pIMU_data.dEuler_angle_Yaw);
				transform.setRotation(q);
				//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "imu_link"));
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), tf_prefix_ + "/base_link", tf_prefix_ + "/imu_link"));
			}


		}


        
        loop_rate.sleep();
    }

	close (serial_fd);

    return 0;
}
