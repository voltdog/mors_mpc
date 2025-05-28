#include "lcm_data_exchange_rm.hpp"
#include <unistd.h>

#define KT 0.74
#define GEAR_RATIO 10.0

LCMExchanger::LCMExchanger()
{

    if(!servo_state_subscriber.good())
        return;
    if(!imu_subscriber.good())
        return;
    if(!enable_publisher.good())
        return;
    if(!servo_cmd_publisher.good())
        return;

    string config_address = mors_sys::GetEnv("CONFIGPATH");
    config_address += "/channels.yaml";

    YAML::Node channel_config = YAML::LoadFile(config_address);
    // YAML::Node channel_config = YAML::LoadFile("/home/user/mors_mpc_control/config/channels.yaml");
    imu_channel = channel_config["imu_data"].as<string>();
    servo_state_channel = channel_config["servo_state"].as<string>();
    servo_cmd_channel = channel_config["servo_cmd"].as<string>();
    enable_channel = channel_config["enable"].as<string>();

    gear_ratio = GEAR_RATIO;

    leg_controller_en = false;
    leg_controller_reset = true;

    imu_orientation_euler.resize(3);
    servo_pos.resize(12);
    servo_vel.resize(12);
    servo_torq.resize(12);

    servo_pos.setZero(12);
}

void LCMExchanger::start_exchanger()
{
    thImu = make_unique<thread> (&LCMExchanger::imuThread, this);
    thServoState = make_unique<thread> (&LCMExchanger::servoStateThread, this);
}

void LCMExchanger::imuHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan, 
                            const mors_msgs::imu_lcm_data* msg)
{
    // cout << "I got IMU data!" << endl;
    imu_orientation_euler(0) = msg->orientation_euler[0];
    imu_orientation_euler(1) = msg->orientation_euler[1];
    imu_orientation_euler(2) = msg->orientation_euler[2];
}

void LCMExchanger::servoStateHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::servo_state_msg* msg)
{
    // cout << "I got servo state data!" << endl;
    for (int i=0; i<12; i++)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
        {
            gear_ratio = GEAR_RATIO;
            // cout << "hey" << endl;
        }
        else
            gear_ratio = GEAR_RATIO;
        servo_pos(i) = msg->position[i];
        servo_vel(i) = msg->velocity[i];
        servo_torq(i) = msg->torque[i] * KT / gear_ratio;
    }
}

void LCMExchanger::imuThread()
{
    imu_subscriber.subscribe(imu_channel, &LCMExchanger::imuHandler, this);
    while(true)
    {
        imu_subscriber.handle();
        // cout << "imu thread" << endl;
        // sleep(0.001);
    }
}

void LCMExchanger::servoStateThread()
{
    servo_state_subscriber.subscribe(servo_state_channel, &LCMExchanger::servoStateHandler, this);
    while(true)
    {
        servo_state_subscriber.handle();
        // cout << "servo_state thread" << endl;
        // sleep(0.001);
    }
}

void LCMExchanger::sendServoCmd(VectorXd &position, VectorXd &velocity, VectorXd &torque, VectorXd &kp, VectorXd &kd)
{
    for (int i=0; i<12; i++)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
        {
            gear_ratio = GEAR_RATIO;
            // cout << "hey" << endl;
        }
        else
            gear_ratio = GEAR_RATIO;
        // cout << i << ": " << gear_ratio << endl;
        servoCmdMsg.position[i] = position(i);
        servoCmdMsg.velocity[i] = velocity(i);
        servoCmdMsg.torque[i] = torque(i) * gear_ratio / KT;
        servoCmdMsg.kp[i] = kp(i);
        servoCmdMsg.kd[i] = kd(i);
    }
    // for (int i=0; i<12; i++)
    // {
    //     servoCmdMsg.position[i] = 0.0;
    //     servoCmdMsg.velocity[i] = 0.0;
    //     servoCmdMsg.torque[i] = 0.0;
    //     servoCmdMsg.kp[i] = 0.0;
    //     servoCmdMsg.kd[i] = 0.0;
    // }
    // cout << torque << endl;
    // servoCmdMsg.torque[2] = 1.0;
    servo_cmd_publisher.publish(servo_cmd_channel, &servoCmdMsg);
}

void LCMExchanger::sendEnableData(bool &locomotion_en, bool &locomotion_reset,
                                    bool &leg_controller_en, bool &leg_controller_reset)
{
    enableMsg.locomotion_en = locomotion_en;
    enableMsg.locomotion_reset = locomotion_reset;

    enableMsg.leg_controller_en = leg_controller_en;
    enableMsg.leg_controller_reset = leg_controller_reset;

    enable_publisher.publish(enable_channel, &enableMsg);
}

void LCMExchanger::getImuData(VectorXd &orientation_euler)
{
    orientation_euler = imu_orientation_euler;
}

void LCMExchanger::getServoStateData(VectorXd &position, VectorXd &velocity, VectorXd &torque)
{
    position = servo_pos;
    velocity = servo_vel;
    torque = servo_torq;
}

LCMExchanger::~LCMExchanger()
{
    bool en = false;
    bool reset = true;
    sendEnableData(en, reset, en, reset);
}
