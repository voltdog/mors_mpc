#include "lcm_data_exchange_ml.hpp"
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
    if(!servo_cmd_subscriber.good())
        return;
    if(!odometry_subscriber.good())
        return;
    if(!servo_state_filt_subscriber.good())
        return;
    if(!robot_state_subscriber.good())
        return;
    if(!robot_cmd_subscriber.good())
        return;
    if(!phase_sig_subscriber.good())
        return;

    // char cwd[PATH_MAX];
    // if (getcwd(cwd, sizeof(cwd)) != NULL) {
    //     // print the current working directory
    //     // cout << "Current working directory: " << cwd << endl;
    // } 
    string config_address = mors_sys::GetEnv("CONFIGPATH");//cwd;
    config_address += "/channels.yaml";
    // cout << config_address << endl;

    YAML::Node channel_config = YAML::LoadFile(config_address);//"/home/user/mors_mpc_control/config/channels.yaml");
    imu_channel = channel_config["imu_data"].as<string>();
    servo_state_channel = channel_config["servo_state"].as<string>();
    servo_cmd_channel = channel_config["servo_cmd"].as<string>();
    foot_cmd_channel = channel_config["foot_cmd"].as<string>();
    grf_cmd_channel = channel_config["grf_cmd"].as<string>();
    enable_channel = channel_config["enable"].as<string>();
    control_type_channel = channel_config["control_type"].as<string>();
    odometry_channel = channel_config["odometry"].as<string>();
    robot_state_channel = channel_config["robot_state"].as<string>();
    robot_cmd_channel = channel_config["robot_cmd"].as<string>();
    phase_sig_channel = channel_config["gait_phase"].as<string>();

    // gear_ratio = GEAR_RATIO;

    leg_controller_enable = false;
    leg_controller_reset = true;
    locomotion_enable = false;
    locomotion_reset = true;
    action_ctr_enable = false;
    action_ctr_reset = true;
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
    
    servo_state.pos.resize(12);
    servo_state.vel.resize(12);
    servo_state.torq.resize(12);
    servo_state.pos.setZero();
    servo_state.vel.setZero();
    servo_state.torq.setZero();

    servo_state_filt.pos.resize(12);
    servo_state_filt.vel.resize(12);
    servo_state_filt.torq.resize(12);
    servo_state_filt.pos.setZero();
    servo_state_filt.vel.setZero();
    servo_state_filt.torq.setZero();
    
    servo_cmd.pos.resize(12);
    servo_cmd.vel.resize(12);
    servo_cmd.torq.resize(12);
    servo_cmd.kp.resize(12);
    servo_cmd.kd.resize(12);
    servo_cmd.pos.setZero();
    servo_cmd.vel.setZero();
    servo_cmd.torq.setZero();
    servo_cmd.kp.setZero();
    servo_cmd.kd.setZero();

    imu_data.orientation_euler.resize(3);
    imu_data.orientation_quaternion.resize(4);
    imu_data.ang_vel.resize(3);
    imu_data.lin_accel.resize(3);
    imu_data.orientation_euler.setZero();
    imu_data.orientation_quaternion.setZero();
    imu_data.ang_vel.setZero();
    imu_data.lin_accel.setZero();

    odometry.position.resize(3);
    odometry.position.setZero();
    odometry.orientation.resize(3);
    odometry.orientation.setZero();
    odometry.ang_vel.resize(3);
    odometry.ang_vel.setZero();
    odometry.lin_vel.resize(3);
    odometry.lin_vel.setZero();

    leg_cmd.r1_grf << 0.0, 0.0, 0.0;
    leg_cmd.l1_grf << 0.0, 0.0, 0.0;
    leg_cmd.r2_grf << 0.0, 0.0, 0.0;
    leg_cmd.l2_grf << 0.0, 0.0, 0.0;
    leg_cmd.r1_pos.setZero();
    leg_cmd.r2_pos.setZero();
    leg_cmd.l1_pos.setZero();
    leg_cmd.l2_pos.setZero();
    leg_cmd.r1_vel.setZero();
    leg_cmd.r2_vel.setZero();
    leg_cmd.l1_vel.setZero();
    leg_cmd.l2_vel.setZero();
    leg_cmd.r1_acc.setZero();
    leg_cmd.r2_acc.setZero();
    leg_cmd.l1_acc.setZero();
    leg_cmd.l2_acc.setZero();
    leg_cmd.kp.setZero();
    leg_cmd.kd.setZero();

    
    body_state.orientation.resize(3);
    body_state.ang_vel.resize(3);
    body_state.pos.resize(3);
    body_state.lin_vel.resize(3);
    body_state.orientation.setZero();
    body_state.ang_vel.setZero();
    body_state.pos.setZero();
    body_state.lin_vel.setZero();

    leg_state.r1_grf.resize(3);
    leg_state.r2_grf.resize(3);
    leg_state.l1_grf.resize(3);
    leg_state.l2_grf.resize(3);
    leg_state.r1_pos.resize(3);
    leg_state.r2_pos.resize(3);
    leg_state.l1_pos.resize(3);
    leg_state.l2_pos.resize(3);
    leg_state.r1_vel.resize(3);
    leg_state.r2_vel.resize(3);
    leg_state.l1_vel.resize(3);
    leg_state.l2_vel.resize(3);
    leg_state.r1_acc.resize(3);
    leg_state.r2_acc.resize(3);
    leg_state.l1_acc.resize(3);
    leg_state.l2_acc.resize(3);
    leg_state.contacts.resize(4);

    leg_state.r1_grf.setZero();
    leg_state.l1_grf.setZero();
    leg_state.r2_grf.setZero();
    leg_state.l2_grf.setZero();
    leg_state.r1_pos.setZero();
    leg_state.r2_pos.setZero();
    leg_state.l1_pos.setZero();
    leg_state.l2_pos.setZero();
    leg_state.r1_vel.setZero();
    leg_state.r2_vel.setZero();
    leg_state.l1_vel.setZero();
    leg_state.l2_vel.setZero();
    leg_state.r1_acc.setZero();
    leg_state.r2_acc.setZero();
    leg_state.l1_acc.setZero();
    leg_state.l2_acc.setZero();
    leg_state.contacts = {false, false, false, false};

    phase.setZero();
    phi.setZero();

    body_cmd.ang_vel.resize(3);
    body_cmd.lin_vel.resize(3);
    body_cmd.pos.resize(3);
    body_cmd.orientation.resize(3);
    body_cmd.ang_vel.setZero();
    body_cmd.lin_vel.setZero();
    body_cmd.pos.setZero();
    body_cmd.orientation.setZero();

    // double bx = 0.165;
    // double by = 0.067;
    // r1_pos <<  bx, -by, -0.18;
    // leg_cmd.r1_pos <<  0.225, -0.1, -0.17;
    // leg_cmd.l1_pos <<  0.225,  0.1, -0.17;
    // leg_cmd.r2_pos << -0.225, -0.1, -0.17;
    // leg_cmd.l2_pos << -0.225,  0.1, -0.17;

    // leg_cmd.r1_vel << 0.0, 0.0, 0.0;
    // leg_cmd.l1_vel << 0.0, 0.0, 0.0;
    // leg_cmd.r2_vel << 0.0, 0.0, 0.0;
    // leg_cmd.l2_vel << 0.0, 0.0, 0.0;
}

void LCMExchanger::start_exchanger()
{
    thFootCmd = make_unique<thread> (&LCMExchanger::footCmdThread, this);
    thGrfCmd = make_unique<thread> (&LCMExchanger::grfCmdThread, this);
    thImu = make_unique<thread> (&LCMExchanger::imuThread, this);
    thServoState = make_unique<thread> (&LCMExchanger::servoStateThread, this);
    thServoCmd = make_unique<thread> (&LCMExchanger::servoCmdThread, this);
    thEnable = make_unique<thread> (&LCMExchanger::enableThread, this);
    thCtrlType = make_unique<thread> (&LCMExchanger::ctrlTypeThread, this);
    thOdometry = make_unique<thread> (&LCMExchanger::odometryThread, this);
    thServoStateFilt = make_unique<thread> (&LCMExchanger::servoStateFiltThread, this);
    thRobotState = make_unique<thread> (&LCMExchanger::robotStateThread, this);
    thRobotCmd = make_unique<thread> (&LCMExchanger::robotCmdThread, this);
    thPhaseSig = make_unique<thread> (&LCMExchanger::phaseSigThread, this);
}

void LCMExchanger::footCmdHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::foot_cmd_msg* msg)
{
    // cout << "I got foot_cmd!" << endl;
    for (int i=0; i<3; i++)
    {
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
        servo_state.torq(i) = -msg->torque[i];// * KT / gear_ratio;
    }
}

void LCMExchanger::servoStateFiltHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const mors_msgs::servo_state_msg* msg)
{
// cout << "I got servo state data!" << endl;
    for (int i=0; i<12; i++)
    {
        servo_state_filt.pos(i) = msg->position[i];
        servo_state_filt.vel(i) = msg->velocity[i];
        servo_state_filt.torq(i) = -msg->torque[i];// * KT / gear_ratio;
    }
}

void LCMExchanger::servoCmdHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const mors_msgs::servo_cmd_msg* msg)
{
    // cout << "I got servo state data!" << endl;
    for (int i=0; i<12; i++)
    {
        servo_cmd.pos(i) = msg->position[i];
        servo_cmd.vel(i) = msg->velocity[i];
        servo_cmd.torq(i) = msg->torque[i];// * KT / gear_ratio;
        servo_cmd.kp(i) = msg->kp[i];
        servo_cmd.kd(i) = msg->kd[i];
    }
}

void LCMExchanger::enableHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const mors_msgs::enable_msg* msg)
{
    // cout << "I got ENABLE data!" << endl;
    leg_controller_enable = msg->leg_controller_en;
    leg_controller_reset = msg->leg_controller_reset;

    locomotion_enable = msg->locomotion_en;
    locomotion_reset = msg->locomotion_reset;

    action_ctr_enable = msg->action_ctr_en;
    action_ctr_reset = msg->action_ctr_reset;
}

void LCMExchanger::ctrlTypeHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const mors_msgs::std_int*msg)
{
    cout << "I got CONTROL_TYPE data!" << endl;
    control_type = msg->data;
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

void LCMExchanger::robotStateHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const mors_msgs::robot_state_msg* msg)
{
    // cout << "I got robot_state!" << endl;
    for (int i=0; i<3; i++)
    {
        leg_state.r1_pos(i) = msg->legs.r1_pos[i];
        leg_state.l1_pos(i) = msg->legs.l1_pos[i];
        leg_state.r2_pos(i) = msg->legs.r2_pos[i];
        leg_state.l2_pos(i) = msg->legs.l2_pos[i];

        leg_state.r1_vel(i) = msg->legs.r1_vel[i];
        leg_state.l1_vel(i) = msg->legs.l1_vel[i];
        leg_state.r2_vel(i) = msg->legs.r2_vel[i];
        leg_state.l2_vel(i) = msg->legs.l2_vel[i];

        leg_state.r1_grf(i) = msg->legs.r1_grf[i];
        leg_state.l1_grf(i) = msg->legs.l1_grf[i];
        leg_state.r2_grf(i) = msg->legs.r2_grf[i];
        leg_state.l2_grf(i) = msg->legs.l2_grf[i];

        leg_state.contacts[i] = msg->legs.contact_states[i];

        body_state.pos(i) = msg->body.position[i];
        body_state.orientation(i) = msg->body.orientation[i];
        body_state.lin_vel(i) = msg->body.lin_vel[i];
        body_state.ang_vel(i) = msg->body.ang_vel[i];
    }
    leg_state.contacts[3] = msg->legs.contact_states[3];
}

void LCMExchanger::robotCmdHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const mors_msgs::robot_cmd_msg* msg)
{
    // cout << "I got robot_cmd!" << endl;
    for (int i=0; i<3; i++)
    {
        body_cmd.pos(i) = msg->cmd_pose[i];
        body_cmd.orientation(i) = msg->cmd_pose[i+3];
        body_cmd.lin_vel(i) = msg->cmd_vel[i];
        body_cmd.ang_vel(i) = msg->cmd_vel[i+3];
    }
}

void LCMExchanger::phaseSigHandler(const lcm::ReceiveBuffer* rbuf, 
    const std::string& chan, 
    const mors_msgs::phase_signal_msg* msg)
{
    for (int i=0; i<4; i++)
    {
        phase(i) = msg->phase[i];
        phi(i) = msg->phi[i];
    }
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

void LCMExchanger::servoStateFiltThread()
{
    servo_state_filt_subscriber.subscribe(SERVO_FILTERED_CHANNEL, &LCMExchanger::servoStateFiltHandler, this);
    while(true)
    {
        servo_state_filt_subscriber.handle();
        // cout << "servo_state thread" << endl;
        // sleep(0.001);
    }
}

void LCMExchanger::servoCmdThread()
{
    servo_cmd_subscriber.subscribe(servo_cmd_channel, &LCMExchanger::servoCmdHandler, this);
    while(true)
    {
        servo_cmd_subscriber.handle();
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

void LCMExchanger::odometryThread()
{   
    odometry_subscriber.subscribe(odometry_channel, &LCMExchanger::odometryHandler, this);
    while(true)
    {
        odometry_subscriber.handle();
        // cout << "grf thread" << endl;
        // sleep(0.001);
    }
}

void LCMExchanger::robotStateThread()
{
    robot_state_subscriber.subscribe(robot_state_channel, &LCMExchanger::robotStateHandler, this);
    while(true)
    {
        robot_state_subscriber.handle();
        // cout << "robot_state thread" << endl;
        // sleep(0.001);
    }
}

void LCMExchanger::robotCmdThread()
{
    robot_cmd_subscriber.subscribe(robot_cmd_channel, &LCMExchanger::robotCmdHandler, this);
    while(true)
    {
        robot_cmd_subscriber.handle();
        // cout << "robot_state thread" << endl;
        // sleep(0.001);
    }
}

void LCMExchanger::phaseSigThread()
{
    phase_sig_subscriber.subscribe(phase_sig_channel, &LCMExchanger::phaseSigHandler, this);
    while(true)
    {
        phase_sig_subscriber.handle();
        // cout << "robot_state thread" << endl;
        // sleep(0.001);
    }
}

LegData LCMExchanger::getLegCmdData()
{
    return leg_cmd;
}

ImuData LCMExchanger::getImuData()
{
    return imu_data;
}

ServoData LCMExchanger::getServoStateData()
{
    return servo_state;
}

ServoData LCMExchanger::getServoStateFiltData()
{
    return servo_state_filt;
}

ServoData LCMExchanger::getServoCmdData()
{
    return servo_cmd;
}

Odometry LCMExchanger::getOdometry()
{
    return odometry;
}

RobotData LCMExchanger::getBodyState()
{
    return body_state;
}

LegData LCMExchanger::getLegState()
{
    return leg_state;
}

RobotData LCMExchanger::getRobotCmd()
{
    return body_cmd;
}
void LCMExchanger::getPhaseSig(Vector4i& phase, Vector4d& phi)
{
    phase = this->phase;
    phi = this->phi;
}

void LCMExchanger::getEnableData(bool &leg_controller_en, bool &leg_controller_reset,
                                bool &locomotion_en, bool &locomotion_reset,
                                bool &action_ctr_en, bool &action_ctr_reset)
{
    leg_controller_en = this->leg_controller_enable;
    leg_controller_reset = this->leg_controller_reset;

    locomotion_en = this->locomotion_enable;
    locomotion_reset = this->locomotion_reset;

    action_ctr_en = this->action_ctr_enable;
    action_ctr_reset = this->action_ctr_reset;
}

int LCMExchanger::getCtrlTypeData()
{
    return control_type;
}

LCMExchanger::~LCMExchanger()
{

}
