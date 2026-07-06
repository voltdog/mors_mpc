#include "lcm_data_exchange_se.hpp"
#include <cstdlib>
#include <stdexcept>
#include <unistd.h>

namespace
{

std::string GetRequiredEnv(const char* name)
{
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0')
    {
        throw std::runtime_error(std::string("[StateEstimator] ") + name + " must be set.");
    }
    return value;
}

} // namespace

LCMExchanger::LCMExchanger()
    : servo_state_subscriber(GetRequiredEnv("LCM_SERVO_URL")),
      imu_subscriber(GetRequiredEnv("LCM_CONTROL_URL")),
      odometry_subscriber(GetRequiredEnv("LCM_CONTROL_URL")),
      contact_subscriber(GetRequiredEnv("LCM_CONTROL_URL")),
      robot_state_publisher(GetRequiredEnv("LCM_CONTROL_URL")),
      servo_filtered_publisher(GetRequiredEnv("LCM_SERVO_URL"))
{
    
    if(!servo_state_subscriber.good())
        return;
    if(!imu_subscriber.good())
        return;
    if(!odometry_subscriber.good())
        return;
    if(!contact_subscriber.good())
        return;
    if(!robot_state_publisher.good())
        return;
    if(!servo_filtered_publisher.good())
        return;

    string config_address = mors_sys::GetEnv("CONFIGPATH");//cwd;
    config_address += "/channels.yaml";

    YAML::Node channel_config = YAML::LoadFile(config_address);//"/home/user/mors_mpc_control/config/channels.yaml");
    imu_channel = channel_config["imu_data"].as<string>();
    servo_state_channel = channel_config["servo_state"].as<string>();
    odometry_channel = channel_config["odometry"].as<string>();
    robot_state_channel = channel_config["robot_state"].as<string>(); //"ROBOT_STATE_CHECK"; //
    contact_channel = channel_config["contact_state"].as<string>();

    // imu_data.orientation_euler.resize(3);
    imu_data.orientation_euler.setZero();
    imu_data.orientation_quaternion.setZero();
    // imu_data.ang_vel.resize(3);
    imu_data.ang_vel.setZero();
    // imu_data.lin_accel.resize(3);
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
    odometry.orientation_euler.resize(3);
    odometry.orientation_euler.setZero();
    odometry.orientation_quaternion.setZero();
    odometry.ang_vel.resize(3);
    odometry.ang_vel.setZero();

    contact_states.resize(4);
    contact_states[0] = false;
    contact_states[1] = false;
    contact_states[2] = false;
    contact_states[3] = false;
}

void LCMExchanger::start_exchanger()
{
    thImu = make_unique<thread> (&LCMExchanger::imuThread, this);
    thServoState = make_unique<thread> (&LCMExchanger::servoStateThread, this);
    thOdometry = make_unique<thread> (&LCMExchanger::odometryThread, this);
    thContact = make_unique<thread> (&LCMExchanger::contactThread, this);
}


void LCMExchanger::imuHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan, 
    const mors_msgs::imu_lcm_data* msg)
{
    // cout << "I got IMU data!" << endl;
    for (int i=0; i<3; i++)
    {
        imu_data.orientation_euler(i) = msg->orientation_euler[i];
        imu_data.orientation_quaternion(i) = msg->orientation_quaternion[i];
        imu_data.ang_vel(i) = msg->angular_velocity[i];
        imu_data.lin_accel(i) = msg->linear_acceleration[i];
    }
    imu_data.orientation_quaternion(3) = msg->orientation_quaternion[3];
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
        odometry.orientation_euler(i) = msg->orientation[i];
        odometry.orientation_quaternion(i) = msg->orientation_quaternion[i];
        odometry.lin_vel(i) = msg->lin_vel[i];
        odometry.ang_vel(i) = msg->ang_vel[i];
    }
    odometry.orientation_quaternion(3) = msg->orientation_quaternion[3];
}


void LCMExchanger::contactHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan, 
    const mors_msgs::contact_sensor_msg* msg)
{
    // cout << "I got CONTACT data!" << endl;
    for (int i=0; i<4; i++)
    {
        contact_states[i] = msg->contact_states[i];
    }
}

void LCMExchanger::contactThread()
{
    contact_subscriber.subscribe(contact_channel, &LCMExchanger::contactHandler, this);
    while(true)
    {
        contact_subscriber.handle();
        // cout << "contact thread" << endl;
        // sleep(0.001);
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

std::vector<bool> LCMExchanger::getContactData()
{
    return contact_states;
}

void LCMExchanger::sendRobotState(RobotData robot_state, LegData leg_state)
{
    for (int i=0; i<3; i++)
    {
        rs_msg.body.position[i] = robot_state.pos[i];
        rs_msg.body.orientation[i] = robot_state.orientation[i];
        rs_msg.body.orientation_quaternion[i] = robot_state.orientation_quaternion[i];
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
    rs_msg.body.orientation_quaternion[3] = robot_state.orientation_quaternion[3];

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
