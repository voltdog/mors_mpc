#include "lcm_data_exchange_lc.hpp"
#include <unistd.h>

// #define KT 0.74
// #define GEAR_RATIO 10.0

LCMExchanger::LCMExchanger()
{
    
    if(!wbc_cmd_subscriber.good())
        return;
    if(!servo_state_subscriber.good())
        return;
    if(!imu_subscriber.good())
        return;
    if(!enable_subscriber.good())
        return;
    if(!phase_sig_subscriber.good())
        return;
    if(!servo_cmd_publisher.good())
        return;

    string config_address = mors_sys::GetEnv("CONFIGPATH");//cwd;
    config_address += "/channels.yaml";

    YAML::Node channel_config = YAML::LoadFile(config_address);
    enable_channel = channel_config["enable"].as<string>();
    imu_channel = channel_config["imu_data"].as<string>();
    wbc_cmd_channel = channel_config["wbc_cmd"].as<string>();
    servo_cmd_channel = channel_config["servo_cmd"].as<string>();
    servo_state_channel = channel_config["servo_state"].as<string>();
    phase_sig_channel = channel_config["gait_phase"].as<string>();

    enable = false;
    reset = true;

    servo_pos.resize(12);
    servo_vel.resize(12);
    servo_torq.resize(12);

    servo_pos.setZero(12);
    phase_signal << 1, 1, 1, 1;

    leg_cmd.r1_pos <<  0.225, -0.1, -0.17;
    leg_cmd.l1_pos <<  0.225,  0.1, -0.17;
    leg_cmd.r2_pos << -0.225, -0.1, -0.17;
    leg_cmd.l2_pos << -0.225,  0.1, -0.17;
}

void LCMExchanger::start_exchanger()
{
    thWbcCmd = make_unique<thread> (&LCMExchanger::wbcCmdThread, this);
    thImu = make_unique<thread> (&LCMExchanger::imuThread, this);
    thServoState = make_unique<thread> (&LCMExchanger::servoStateThread, this);
    thEnable = make_unique<thread> (&LCMExchanger::enableThread, this);
    thPhaseSig = make_unique<thread> (&LCMExchanger::phaseSigThread, this);
}

void LCMExchanger::wbcCmdHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::wbc_cmd_msg* msg)
{
    // cout << "I got wbc_cmd!" << endl;
    for (int i=0; i<3; i++)
    {
        leg_cmd.r1_pos(i) = msg->legs.r1_pos[i];
        leg_cmd.l1_pos(i) = msg->legs.l1_pos[i];
        leg_cmd.r2_pos(i) = msg->legs.r2_pos[i];
        leg_cmd.l2_pos(i) = msg->legs.l2_pos[i];

        leg_cmd.r1_vel(i) = msg->legs.r1_vel[i];
        leg_cmd.l1_vel(i) = msg->legs.l1_vel[i];
        leg_cmd.r2_vel(i) = msg->legs.r2_vel[i];
        leg_cmd.l2_vel(i) = msg->legs.l2_vel[i];

        leg_cmd.r1_grf(i) = msg->legs.r1_grf[i];
        leg_cmd.r2_grf(i) = msg->legs.r2_grf[i];
        leg_cmd.l1_grf(i) = msg->legs.l1_grf[i];
        leg_cmd.l2_grf(i) = msg->legs.l2_grf[i];
    }
}

void LCMExchanger::imuHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan, 
                            const mors_msgs::imu_lcm_data* msg)
{
    imu_orientation_euler(0) = msg->orientation_euler[0];
    imu_orientation_euler(1) = msg->orientation_euler[1];
    imu_orientation_euler(2) = msg->orientation_euler[2];
}

void LCMExchanger::servoStateHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::servo_state_msg* msg)
{
    for (int i=0; i<12; i++)
    {
        servo_pos(i) = msg->position[i];
        servo_vel(i) = msg->velocity[i];
        servo_torq(i) = msg->torque[i];// * KT / gear_ratio;
    }
}

void LCMExchanger::enableHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const mors_msgs::enable_msg* msg)
{
    // cout << "I got ENABLE data!" << endl;
    enable = msg->leg_controller_en;
    reset = msg->leg_controller_reset;
}

void LCMExchanger::phaseSigHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const mors_msgs::phase_signal_msg* msg)
{
    // cout << "I got CONTROL_TYPE data!" << endl;
    for (int i = 0; i < 4; i++)
        phase_signal(i) = msg->phase[i];
}

void LCMExchanger::wbcCmdThread()
{   
    wbc_cmd_subscriber.subscribe(wbc_cmd_channel, &LCMExchanger::wbcCmdHandler, this);
    while(true)
    {
        wbc_cmd_subscriber.handle();
    }
}


void LCMExchanger::imuThread()
{
    imu_subscriber.subscribe(imu_channel, &LCMExchanger::imuHandler, this);
    while(true)
    {
        imu_subscriber.handle();
    }
}

void LCMExchanger::servoStateThread()
{
    servo_state_subscriber.subscribe(servo_state_channel, &LCMExchanger::servoStateHandler, this);
    while(true)
    {
        servo_state_subscriber.handle();
    }
}

void LCMExchanger::enableThread()
{
    enable_subscriber.subscribe(enable_channel, &LCMExchanger::enableHandler, this);
    while(true)
    {
        enable_subscriber.handle();
    }
}

void LCMExchanger::phaseSigThread()
{
    phase_sig_subscriber.subscribe(phase_sig_channel, &LCMExchanger::phaseSigHandler, this);
    while(true)
    {
        phase_sig_subscriber.handle();
    }
}


void LCMExchanger::sendServoCmd(VectorXd &position, VectorXd &velocity, VectorXd &torque, VectorXd &kp, VectorXd &kd)
{
    for (int i=0; i<12; i++)
    {
        servoCmdMsg.position[i] = position(i);
        servoCmdMsg.velocity[i] = velocity(i);
        servoCmdMsg.torque[i] = torque(i);// * gear_ratio / KT;
        servoCmdMsg.kp[i] = kp(i);
        servoCmdMsg.kd[i] = kd(i);
    }

    servo_cmd_publisher.publish(servo_cmd_channel, &servoCmdMsg);
}

void LCMExchanger::getLegCmdData(LegData &leg_cmd)
{
    leg_cmd.r1_grf = this->leg_cmd.r1_grf;
    leg_cmd.l1_grf = this->leg_cmd.l1_grf;
    leg_cmd.r2_grf = this->leg_cmd.r2_grf;
    leg_cmd.l2_grf = this->leg_cmd.l2_grf;

    leg_cmd.r1_pos = this->leg_cmd.r1_pos;
    leg_cmd.l1_pos = this->leg_cmd.l1_pos;
    leg_cmd.r2_pos = this->leg_cmd.r2_pos;
    leg_cmd.l2_pos = this->leg_cmd.l2_pos;

    leg_cmd.r1_vel = this->leg_cmd.r1_vel;
    leg_cmd.l1_vel = this->leg_cmd.l1_vel;
    leg_cmd.r2_vel = this->leg_cmd.r2_vel;
    leg_cmd.l2_vel = this->leg_cmd.l2_vel;
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

void LCMExchanger::getEnableData(bool &en, bool &reset)
{
    en = this->enable;
    reset = this->reset;
}

Vector4i LCMExchanger::getPhaseSignal()
{
    return phase_signal;
}

LCMExchanger::~LCMExchanger()
{
    for (int i=0; i<12; i++)
    {
        servoCmdMsg.position[i] = 0.0;
        servoCmdMsg.velocity[i] = 0.0;
        servoCmdMsg.torque[i] = 0.0;// * gear_ratio / KT;
        servoCmdMsg.kp[i] = 0.0;
        servoCmdMsg.kd[i] = 0.3;
    }

    servo_cmd_publisher.publish(servo_cmd_channel, &servoCmdMsg);
}
