#include <lcm/lcm-cpp.hpp>
#include "mors_msgs/imu_lcm_data.hpp"

#include <sstream>
#include <iostream>

#include <stdio.h> // printf
#include <wchar.h> // wchar_t
#include <cmath> 
#include <yaml-cpp/yaml.h>

#include "hidapi/hidapi.h"

using namespace std;
using namespace YAML;

int open_imu_hid(void);
void quaternionToEuler(float w, float x, float y, float z, float& roll, float& pitch, float& yaw) ;

#define MAX_STR 255
#define X 0
#define Y 1
#define Z 2
#define W 3

#define IMU_CHANNEL "IMU_DATA"

hid_device *acc_handle;

float yaw_offset = 0.0;
bool first = true;

void quaternionToEuler(float w, float x, float y, float z, float& roll, float& pitch, float& yaw) {
    // Roll (вращение вокруг оси X)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

    // Pitch (вращение вокруг оси Y)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1) {
        pitch = std::copysign(M_PI / 2, sinp); // Используем ±90 градусов, если значение выходит за пределы
    } else {
        pitch = asin(sinp);
    }

    // Yaw (вращение вокруг оси Z)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
}

int open_imu_hid()
{
	int res;
	// unsigned char buf[65];
	wchar_t wstr[MAX_STR];

	// int i;

	// Initialize the hidapi library
	res = hid_init();
	
	struct hid_device_info *imu_info;
	imu_info = hid_enumerate(0xCafe, 0x4004);

	cout << "First interface path is " << imu_info->path << endl;

	acc_handle = hid_open_path(imu_info->path);
	if (!acc_handle) {
		cout << "Unable to open IMU!" << endl;
		hid_exit();
 		return 1;
	}
	
	//hid_set_nonblocking(acc_handle, 1);

	cout << "IMU interface is opened successfully!" << endl;

	// Read the Manufacturer String
	res = hid_get_manufacturer_string(acc_handle, wstr, MAX_STR);
	wcout << "Manufacturer String: " << wstr << endl;

	// Read the Product String
	res = hid_get_product_string(acc_handle, wstr, MAX_STR);
	wcout << "Product String: " << wstr << endl;

	// Read the Serial Number String
	res = hid_get_serial_number_string(acc_handle, wstr, MAX_STR);
	wcout << "Serial Number String: " << wstr[0] << wstr << endl;
	
	return 0;
}

int main(int argc, char **argv)
{
	cout << "BHI360_IMU starting..." << endl;
	string imu_channel = IMU_CHANNEL;
	YAML::Node config = YAML::LoadFile("/home/user/mors_experiments/config/channels.yaml");
	imu_channel = config["imu_data"].as<string>();

	float roll = 0.0;
	float pitch = 0.0;
	float yaw = 0.0;
	
	lcm::LCM lcm;
	if(!lcm.good())
		return 1;
	mors_msgs::imu_lcm_data imu_msg;
  
	uint8_t device_present = 1;
	
	if( open_imu_hid() )
		return 1;

	int count = 0;

	cout << "BHI360_IMU started" << endl;
	while (true)
	{
		uint8_t buffer[64];
		int8_t bytes_number = 0;
		
		while( device_present )
		{
			bytes_number = hid_read(acc_handle, buffer, 64);
			
			if( bytes_number < 0 )
			{
				device_present = 0;
			}
			else if( bytes_number )
			{
				int16_t acc_x = *(int16_t*)&buffer[2];
				int16_t acc_y = *(int16_t*)&buffer[4];
				int16_t acc_z = *(int16_t*)&buffer[6];
				
				imu_msg.linear_acceleration[X] = acc_x * 9.81f * 1.0f / 4096.0f;
				imu_msg.linear_acceleration[Y] = acc_y * 9.81f * 1.0f / 4096.0f;
				imu_msg.linear_acceleration[Z] = acc_z * 9.81f * 1.0f / 4096.0f;
				
				int16_t gyr_x = *(int16_t*)&buffer[8];
				int16_t gyr_y = *(int16_t*)&buffer[10];
				int16_t gyr_z = *(int16_t*)&buffer[12];
				
				imu_msg.angular_velocity[X] = gyr_x * 6.28f * 2000.0f / 32768.0f / 360.0f;
				imu_msg.angular_velocity[Y] = gyr_y * 6.28f * 2000.0f / 32768.0f / 360.0f;
				imu_msg.angular_velocity[Z] = gyr_z * 6.28f * 2000.0f / 32768.0f / 360.0f;	
				
				int16_t quat_x = *(int16_t*)&buffer[14];
				int16_t quat_y = *(int16_t*)&buffer[16];
				int16_t quat_z = *(int16_t*)&buffer[18];
				int16_t quat_w = *(int16_t*)&buffer[20];
				
				imu_msg.orientation_quaternion[X] = quat_x * 1.0f / 16384.0f;
				imu_msg.orientation_quaternion[Y] = quat_y * 1.0f / 16384.0f;
				imu_msg.orientation_quaternion[Z] = quat_z * 1.0f / 16384.0f;
				imu_msg.orientation_quaternion[W] = quat_w * 1.0f / 16384.0f;

				quaternionToEuler(imu_msg.orientation_quaternion[W], imu_msg.orientation_quaternion[X], imu_msg.orientation_quaternion[Y], imu_msg.orientation_quaternion[Z], roll, pitch, yaw);

				// if (first)
				// {
				// 	first = false;
				// 	yaw_offset = yaw;
				// }

				imu_msg.orientation_euler[X] = roll;
				imu_msg.orientation_euler[Y] = pitch;
				imu_msg.orientation_euler[Z] = yaw - yaw_offset;
				
				break ;	
								
			}
			else
			{
				break ;
			}
		}
		
		if( !device_present )
		{
			cout << "USB error!" << endl;
			return 0;
		}

		//ROS_INFO("%s", msg.data.c_str());

		lcm.publish(imu_channel, &imu_msg);

		//probably it is needed to add pause

		++count;
	}


	return 0;
}

