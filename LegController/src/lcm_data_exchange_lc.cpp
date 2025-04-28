#include "lcm_data_exchange_lc.hpp"
#include <unistd.h>

// #define KT 0.74
// #define GEAR_RATIO 10.0

LCMExchanger::LCMExchanger()
{
    
    if(!foot_cmd_subscriber.good())
        return;
    if(!grf_cmd_subscriber.good())
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

    // char cwd[PATH_MAX];
    // if (getcwd(cwd, sizeof(cwd)) != NULL) {
    //     // print the current working directory
    //     // cout << "Current working directory: " << cwd << endl;
    // } 
    string config_address = mors_sys::GetEnv("CONFIGPATH");//cwd;
    config_address += "/channels.yaml";

    YAML::Node channel_config = YAML::LoadFile(config_address);//"/home/user/mors_mpc_control/config/channels.yaml");
    imu_channel = channel_config["imu_data"].as<string>();
    servo_state_channel = channel_config["servo_state"].as<string>();
    servo_cmd_channel = channel_config["servo_cmd"].as<string>();
    foot_cmd_channel = channel_config["foot_cmd"].as<string>();
    grf_cmd_channel = channel_config["grf_cmd"].as<string>();
    enable_channel = channel_config["enable"].as<string>();
    control_type_channel = channel_config["control_type"].as<string>();
    phase_sig_channel = channel_config["gait_phase"].as<string>();

    // gear_ratio = GEAR_RATIO;

    enable = false;
    reset = true;
    control_type = 0;

    leg_cmd.r1_grf.resize(3);
    leg_cmd.r2_grf.resize(3);
    leg_cmd.l1_grf.resize(3);
    leg_cmd.l2_grf.resize(3);
    leg_cmd.r1_pos.resize(3);
    leg_cmd.r2_pos.resize(3);
    leg_cmd.l1_pos.resize(3);
    leg_cmd.l2_pos.resize(3);
    leg_cmd.r1_vel.resize(3);
    leg_cmd.r2_vel.resize(3);
    leg_cmd.l1_vel.resize(3);
    leg_cmd.l2_vel.resize(3);
    leg_cmd.r1_acc.resize(3);
    leg_cmd.r2_acc.resize(3);
    leg_cmd.l1_acc.resize(3);
    leg_cmd.l2_acc.resize(3);
    leg_cmd.kp.resize(3);
    leg_cmd.kd.resize(3);
    imu_orientation_euler.resize(3);
    servo_pos.resize(12);
    servo_vel.resize(12);
    servo_torq.resize(12);

    servo_pos.setZero(12);
    leg_cmd.r1_grf << 0.0, 0.0, 0.0;
    leg_cmd.l1_grf << 0.0, 0.0, 0.0;
    leg_cmd.r2_grf << 0.0, 0.0, 0.0;
    leg_cmd.l2_grf << 0.0, 0.0, 0.0;
    leg_cmd.kp.setZero();
    leg_cmd.kd.setZero();
    phase_signal << 1, 1, 1, 1;

    // double bx = 0.165;
    // double by = 0.067;
    // r1_pos <<  bx, -by, -0.18;
    leg_cmd.r1_pos <<  0.225, -0.1, -0.17;
    leg_cmd.l1_pos <<  0.225,  0.1, -0.17;
    leg_cmd.r2_pos << -0.225, -0.1, -0.17;
    leg_cmd.l2_pos << -0.225,  0.1, -0.17;

    leg_cmd.r1_vel << 0.0, 0.0, 0.0;
    leg_cmd.l1_vel << 0.0, 0.0, 0.0;
    leg_cmd.r2_vel << 0.0, 0.0, 0.0;
    leg_cmd.l2_vel << 0.0, 0.0, 0.0;
}

void LCMExchanger::start_exchanger()
{
    thFootCmd = make_unique<thread> (&LCMExchanger::footCmdThread, this);
    thGrfCmd = make_unique<thread> (&LCMExchanger::grfCmdThread, this);
    thImu = make_unique<thread> (&LCMExchanger::imuThread, this);
    thServoState = make_unique<thread> (&LCMExchanger::servoStateThread, this);
    thEnable = make_unique<thread> (&LCMExchanger::enableThread, this);
    thCtrlType = make_unique<thread> (&LCMExchanger::ctrlTypeThread, this);
    thPhaseSig = make_unique<thread> (&LCMExchanger::phaseSigThread, this);
}

void LCMExchanger::footCmdHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::foot_cmd_msg* msg)
{
    // cout << "I got foot_cmd!" << endl;
    for (int i=0; i<3; i++)
    {
        // r1_grf(i) = msg->r1_grf[i];
        // r2_grf(i) = msg->r2_grf[i];
        // l1_grf(i) = msg->l1_grf[i];
        // l2_grf(i) = msg->l2_grf[i];

        leg_cmd.r1_pos(i) = msg->r1_pos[i];
        leg_cmd.l1_pos(i) = msg->l1_pos[i];
        leg_cmd.r2_pos(i) = msg->r2_pos[i];
        leg_cmd.l2_pos(i) = msg->l2_pos[i];

        leg_cmd.r1_vel(i) = msg->r1_vel[i];
        leg_cmd.l1_vel(i) = msg->l1_vel[i];
        leg_cmd.r2_vel(i) = msg->r2_vel[i];
        leg_cmd.l2_vel(i) = msg->l2_vel[i];

        leg_cmd.r1_acc(i) = msg->r1_acc[i];
        leg_cmd.l1_acc(i) = msg->l1_acc[i];
        leg_cmd.r2_acc(i) = msg->r2_acc[i];
        leg_cmd.l2_acc(i) = msg->l2_acc[i];

        leg_cmd.kp(i) = msg->Kp[i];
        leg_cmd.kd(i) = msg->Kd[i];
    }
}

void LCMExchanger::grfCmdHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const mors_msgs::grf_cmd_msg* msg)
{
    // cout << "I got grf_cmd!" << endl;
    for (int i=0; i<3; i++)
    {
        leg_cmd.r1_grf(i) = msg->r1_grf[i];
        leg_cmd.r2_grf(i) = msg->r2_grf[i];
        leg_cmd.l1_grf(i) = msg->l1_grf[i];
        leg_cmd.l2_grf(i) = msg->l2_grf[i];
    }
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
        // if (i == 2 || i == 5 || i == 8 || i == 11)
        // {
        //     gear_ratio = GEAR_RATIO;
        //     // cout << "hey" << endl;
        // }
        // else
        //     gear_ratio = GEAR_RATIO;
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

void LCMExchanger::ctrlTypeHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const mors_msgs::std_int*msg)
{
    // cout << "I got CONTROL_TYPE data!" << endl;
    control_type = msg->data;
}

void LCMExchanger::phaseSigHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const mors_msgs::phase_signal_msg* msg)
{
    // cout << "I got CONTROL_TYPE data!" << endl;
    for (int i = 0; i < 4; i++)
        phase_signal(i) = msg->phase[i];
}

void LCMExchanger::footCmdThread()
{   
    foot_cmd_subscriber.subscribe(foot_cmd_channel, &LCMExchanger::footCmdHandler, this);
    while(true)
    {
        foot_cmd_subscriber.handle();
        // cout << "grf thread" << endl;
        // sleep(0.001);
    }
}

void LCMExchanger::grfCmdThread()
{   
    grf_cmd_subscriber.subscribe(grf_cmd_channel, &LCMExchanger::grfCmdHandler, this);
    while(true)
    {
        grf_cmd_subscriber.handle();
        // cout << "grf thread" << endl;
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

void LCMExchanger::enableThread()
{
    enable_subscriber.subscribe(enable_channel, &LCMExchanger::enableHandler, this);
    while(true)
    {
        enable_subscriber.handle();
        // cout << "imu thread" << endl;
        // sleep(0.001);
    }
}

void LCMExchanger::ctrlTypeThread()
{
    controle_type_subscriber.subscribe(control_type_channel, &LCMExchanger::ctrlTypeHandler, this);
    while(true)
    {
        controle_type_subscriber.handle();
        // cout << "imu thread" << endl;
        // sleep(0.001);
    }
}

void LCMExchanger::phaseSigThread()
{
    phase_sig_subscriber.subscribe(phase_sig_channel, &LCMExchanger::phaseSigHandler, this);
    while(true)
    {
        phase_sig_subscriber.handle();
        // cout << "imu thread" << endl;
        // sleep(0.001);
    }
}


void LCMExchanger::sendServoCmd(VectorXd &position, VectorXd &velocity, VectorXd &torque, VectorXd &kp, VectorXd &kd)
{
    for (int i=0; i<12; i++)
    {
        // if (i == 2 || i == 5 || i == 8 || i == 11)
        // {
        //     gear_ratio = GEAR_RATIO;
        //     // cout << "hey" << endl;
        // }
        // else
        //     gear_ratio = GEAR_RATIO;
        // cout << i << ": " << gear_ratio << endl;
        servoCmdMsg.position[i] = position(i);
        servoCmdMsg.velocity[i] = velocity(i);
        servoCmdMsg.torque[i] = torque(i);// * gear_ratio / KT;
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

    leg_cmd.r1_acc = this->leg_cmd.r1_acc;
    leg_cmd.l1_acc = this->leg_cmd.l1_acc;
    leg_cmd.r2_acc = this->leg_cmd.r2_acc;
    leg_cmd.l2_acc = this->leg_cmd.l2_acc;

    leg_cmd.kp = this->leg_cmd.kp;
    leg_cmd.kd = this->leg_cmd.kd;
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

int LCMExchanger::getCtrlTypeData()
{
    return control_type;
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
