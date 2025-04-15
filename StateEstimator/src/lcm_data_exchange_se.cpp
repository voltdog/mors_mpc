#include "lcm_data_exchange_se.hpp"
#include <unistd.h>

LCMExchanger::LCMExchanger()
{
    
    if(!servo_state_subscriber.good())
        return;
    if(!imu_subscriber.good())
        return;
    if(!odometry_subscriber.good())
        return;

    string config_address = mors_sys::GetEnv("CONFIGPATH");//cwd;
    config_address += "/channels.yaml";

    YAML::Node channel_config = YAML::LoadFile(config_address);//"/home/user/mors_mpc_control/config/channels.yaml");
    imu_channel = channel_config["imu_data"].as<string>();
    servo_state_channel = channel_config["servo_state"].as<string>();
    odometry_channel = channel_config["odometry"].as<string>();
    robot_state_channel = channel_config["robot_state"].as<string>();

    imu_data.orientation_euler.resize(3);
    imu_data.orientation_euler.setZero();
    imu_data.ang_vel.resize(3);
    imu_data.ang_vel.setZero();
    imu_data.lin_accel.resize(3);
    imu_data.lin_accel.setZero();

    servo_state.pos.resize(12);
    servo_state.pos.setZero();
    servo_state.vel.resize(12);
    servo_state.vel.setZero();
    servo_state.torq.resize(12);
    servo_state.torq.setZero();

    odometry.position.resize(3);
    odometry.position.setZero();
    odometry.lin_vel.resize(3);
    odometry.lin_vel.setZero();
    odometry.orientation.resize(3);
    odometry.orientation.setZero();
    odometry.ang_vel.resize(3);
    odometry.ang_vel.setZero();
}

void LCMExchanger::start_exchanger()
{
    thImu = make_unique<thread> (&LCMExchanger::imuThread, this);
    thServoState = make_unique<thread> (&LCMExchanger::servoStateThread, this);
    thOdometry = make_unique<thread> (&LCMExchanger::odometryThread, this);
}


void LCMExchanger::imuHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan, 
    const mors_msgs::imu_lcm_data* msg)
{
    // cout << "I got IMU data!" << endl;
    for (int i=0; i<3; i++)
    {
        imu_data.orientation_euler(i) = msg->orientation_euler[i];
        imu_data.ang_vel(i) = msg->angular_velocity[i];
        imu_data.lin_accel(i) = msg->linear_acceleration[i];
    }
}

void LCMExchanger::servoStateHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::servo_state_msg* msg)
{
    // cout << "I got servo state data!" << endl;
    for (int i=0; i<12; i++)
    {
        servo_state.pos(i) = msg->position[i];
        servo_state.vel(i) = msg->velocity[i];
        servo_state.torq(i) = -msg->torque[i];//*0.72/10.0;// * KT / gear_ratio;
    }
}

void LCMExchanger::odometryHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan, 
    const mors_msgs::odometry_msg* msg)
{
    // cout << "I got ODOMETRY data!" << endl;
    for (int i=0; i<3; i++)
    {
        odometry.position(i) = msg->position[i];
        odometry.orientation(i) = msg->orientation[i];
        odometry.lin_vel(i) = msg->lin_vel[i];
        odometry.ang_vel(i) = msg->ang_vel[i];
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

void LCMExchanger::odometryThread()
{
    odometry_subscriber.subscribe(odometry_channel, &LCMExchanger::odometryHandler, this);
    while(true)
    {
        odometry_subscriber.handle();
        // cout << "servo_state thread" << endl;
        // sleep(0.001);
    }
}

ImuData LCMExchanger::getImuData()
{
    return imu_data;
}

ServoData LCMExchanger::getServoStateData()
{
    return servo_state;
}

Odometry LCMExchanger::getOdometry()
{
    return odometry;
}

void LCMExchanger::sendRobotState(RobotData robot_state, LegData leg_state)
{
    for (int i=0; i<3; i++)
    {
        rs_msg.body.position[i] = robot_state.pos[i];
        rs_msg.body.orientation[i] = robot_state.orientation[i];
        rs_msg.body.lin_vel[i] = robot_state.lin_vel[i];
        rs_msg.body.ang_vel[i] = robot_state.ang_vel[i];

        rs_msg.legs.r1_grf[i] = leg_state.r1_grf[i];
        rs_msg.legs.l1_grf[i] = leg_state.l1_grf[i];
        rs_msg.legs.r2_grf[i] = leg_state.r2_grf[i];
        rs_msg.legs.l2_grf[i] = leg_state.l2_grf[i];

        rs_msg.legs.r1_pos[i] = leg_state.r1_pos[i];
        rs_msg.legs.l1_pos[i] = leg_state.l1_pos[i];
        rs_msg.legs.r2_pos[i] = leg_state.r2_pos[i];
        rs_msg.legs.l2_pos[i] = leg_state.l2_pos[i];

        rs_msg.legs.r1_vel[i] = leg_state.r1_vel[i];
        rs_msg.legs.l1_vel[i] = leg_state.l1_vel[i];
        rs_msg.legs.r2_vel[i] = leg_state.r2_vel[i];
        rs_msg.legs.l2_vel[i] = leg_state.l2_vel[i];

        rs_msg.legs.contact_states[i] = leg_state.contacts[i];
    }
    rs_msg.legs.contact_states[3] = leg_state.contacts[3];

    robot_state_publisher.publish(robot_state_channel, &rs_msg);
}

void LCMExchanger::sendServoFiltered(ServoData servo_filtered)
{
    for (int i=0; i<12; i++)
    {
        servo_filtered_msg.position[i] = servo_filtered.pos(i);
        servo_filtered_msg.velocity[i] = servo_filtered.vel(i);
        servo_filtered_msg.torque[i] = servo_filtered.torq(i);
    }
    servo_filtered_publisher.publish(SERVO_FILTERED_CHANNEL, &servo_filtered_msg);
}

LCMExchanger::~LCMExchanger()
{

}
