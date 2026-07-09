#include <lcm/lcm-cpp.hpp>
#include "mors_msgs/imu_lcm_data.hpp"

#include <sstream>
#include <iostream>

#include <stdio.h> // printf
#include <wchar.h> // wchar_t
#include <cmath> 
#include <cstdlib>
#include <stdexcept>
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
#define STARTUP_SKIP_PACKETS 10

#define PITCH_OFFSET -0.0

namespace
{

std::string GetRequiredEnv(const char* name)
{
	const char* value = std::getenv(name);
	if (value == nullptr || value[0] == '\0')
	{
		throw std::runtime_error(std::string("[BHI360_IMU]: ") + name + " must be set.");
	}
	return value;
}

} // namespace

hid_device *acc_handle;

float yaw_offset = 0.0;
bool first = true;

struct Quaternion {
    float w, x, y, z;
};

Quaternion q_offset;
Quaternion q_cur;
bool is_offset_saved = false;

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

// Нормализация кватерниона (чтобы избежать дрейфа из-за погрешностей вычислений)
Quaternion normalize(Quaternion q) {
    float norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (norm > 0.0f) {
        float inv = 1.0f / norm;
        q.w *= inv;
        q.x *= inv;
        q.y *= inv;
        q.z *= inv;
    }
    return q;
}

// Функция извлекает из кватерниона поворот только вокруг оси Z
Quaternion extractZRotation(Quaternion q) {
    // Вектор оси Z в формате кватерниона (0, 0, 0, 1)
    // Проекция кватерниона на ось Z
    
    float norm = sqrt(q.w * q.w + q.z * q.z);
    
    // Защита от деления на ноль (если прибор смотрит ровно вверх/вниз)
    if (norm < 0.0001f) {
        return {1.0f, 0.0f, 0.0f, 0.0f}; // Возвращаем кватернион без поворота
    }
    
    Quaternion q_z;
    q_z.w = q.w / norm;
    q_z.x = 0.0f;
    q_z.y = 0.0f;
    q_z.z = q.z / norm;
    
    return q_z;
}

// Получение обратного кватерниона (для единичных кватернионов это сопряженный)
Quaternion inverse(Quaternion q) {
    Quaternion inv;
    inv.w =  q.w;
    inv.x = -q.x;
    inv.y = -q.y;
    inv.z = -q.z;
    return inv;
}

// Умножение кватернионов (q1 * q2)
// Порядок важен! Это умножение для перехода от локальной СК к глобальной
Quaternion multiply(Quaternion q1, Quaternion q2) {
    Quaternion res;
    res.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    res.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    res.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    res.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return res;
}

Quaternion makePitchCorrection(float pitch_offset) {
    const float half_angle = pitch_offset * 0.5f;
    return {
        cos(half_angle),
        0.0f,
        sin(half_angle),
        0.0f
    };
}

void quat_to_rpy(float w, float x, float y, float z, float& roll, float& pitch, float& yaw)
{
  //from my MATLAB implementation

  //edge case!
  double as = min(double(-2.0 * (x*z - w*y)), 0.99999);
  yaw = atan2(2.0 * (x*y + w*z), w*w + x*x - y*y - z*z);
  pitch = asin(as);
  roll = atan2(2.0 * (y*z + w*x), w*w - x*x - y*y + z*z);
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

	// cout << "First interface path is " << imu_info->path << endl;

	acc_handle = hid_open_path(imu_info->path);
	if (!acc_handle) {
		cout << "[BHI360_IMU]: Unable to open IMU!" << endl;
		hid_exit();
 		return 1;
	}
	
	//hid_set_nonblocking(acc_handle, 1);

	cout << "[BHI360_IMU]: IMU interface is opened successfully!" << endl;

	// Read the Manufacturer String
	res = hid_get_manufacturer_string(acc_handle, wstr, MAX_STR);
	// wcout << "Manufacturer String: " << wstr << endl;

	// Read the Product String
	res = hid_get_product_string(acc_handle, wstr, MAX_STR);
	// wcout << "Product String: " << wstr << endl;

	// Read the Serial Number String
	res = hid_get_serial_number_string(acc_handle, wstr, MAX_STR);
	// wcout << "Serial Number String: " << wstr[0] << wstr << endl;
	
	return 0;
}

int main(int argc, char **argv)
{
	cout << "[BHI360_IMU]: starting..." << endl;
	string imu_channel = IMU_CHANNEL;
	const string config_address = GetRequiredEnv("CONFIGPATH") + "/channels.yaml";
	const string control_lcm_url = GetRequiredEnv("LCM_CONTROL_URL");
	YAML::Node config = YAML::LoadFile(config_address);
	imu_channel = config["imu_data"].as<string>();

	float roll = 0.0;
	float pitch = 0.0;
	float yaw = 0.0;
    const Quaternion q_mount_correction = makePitchCorrection(PITCH_OFFSET);
	
	lcm::LCM lcm(control_lcm_url);
	if(!lcm.good())
		return 1;
	cout << "[BHI360_IMU]: control LCM URL: " << control_lcm_url << endl;
	mors_msgs::imu_lcm_data imu_msg;
  
	uint8_t device_present = 1;
	
	if( open_imu_hid() )
		return 1;

	int count = 0;
	int startup_skip_packets = STARTUP_SKIP_PACKETS;

	cout << "[BHI360_IMU]: started" << endl;
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
				
				
                
                q_cur.x = quat_x * 1.0f / 16384.0f;
                q_cur.y = quat_y * 1.0f / 16384.0f;
                q_cur.z = quat_z * 1.0f / 16384.0f;
                q_cur.w = quat_w * 1.0f / 16384.0f;
                q_cur = normalize(q_cur);
                q_cur = normalize(multiply(q_cur, q_mount_correction));

				// quaternionToEuler(imu_msg.orientation_quaternion[W], imu_msg.orientation_quaternion[X], imu_msg.orientation_quaternion[Y], imu_msg.orientation_quaternion[Z], roll, pitch, yaw);
                // quat_to_rpy(q_cur.w, q_cur.x, q_cur.y, q_cur.z, roll, pitch, yaw);

				if (first == true)// && abs(yaw) >= 0.05)
				{
                    if (startup_skip_packets > 0)
                    {
                        --startup_skip_packets;
                        continue;
                    }

					first = false;
                    q_offset = extractZRotation(q_cur);
                    // cout << "[BHI360_IMU]: yaw offset initialized after "
                    //      << STARTUP_SKIP_PACKETS << " packets" << endl;
                    // cout << q_cur.w << " " << q_cur.x << " " << q_cur.y << " " << q_cur.z << endl;
				}
                // cout << yaw << endl;
                

                Quaternion q_offset_inv = inverse(q_offset);
                Quaternion q_relative = multiply(q_offset_inv, q_cur);
                q_relative = normalize(q_relative);
                quat_to_rpy(q_relative.w, q_relative.x, q_relative.y, q_relative.z, roll, pitch, yaw);

                imu_msg.orientation_quaternion[X] = q_relative.x;
				imu_msg.orientation_quaternion[Y] = q_relative.y;
				imu_msg.orientation_quaternion[Z] = q_relative.z;
				imu_msg.orientation_quaternion[W] = q_relative.w;

				imu_msg.orientation_euler[X] = roll;
				imu_msg.orientation_euler[Y] = pitch;
				imu_msg.orientation_euler[Z] = yaw;
				
				break ;	
								
			}
			else
			{
				break ;
			}
		}
		
		if( !device_present )
		{
			cout << "[BHI360_IMU]: USB error!" << endl;
			return 0;
		}

		//ROS_INFO("%s", msg.data.c_str());

		lcm.publish(imu_channel, &imu_msg);

		//probably it is needed to add pause

		++count;
	}


	return 0;
}
