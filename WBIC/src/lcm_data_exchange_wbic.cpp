#include "lcm_data_exchange_wbic.hpp"
#include <unistd.h>

// #define KT 0.74
// #define GEAR_RATIO 10.0

LCMExchanger::LCMExchanger()
{
    initialized = true;
    if(!wbc_cmd_subscriber.good()) initialized = false;
    if(!robot_state_subscriber.good()) initialized = false;
    if(!servo_state_subscriber.good()) initialized = false;
    if(!enable_subscriber.good()) initialized = false;
    if(!gait_phase_sig_subscriber.good()) initialized = false;
    if(!servo_cmd_publisher.good()) initialized = false;
    if(!wbc_state_publisher.good()) initialized = false;

    string config_address = mors_sys::GetEnv("CONFIGPATH");//cwd;
    config_address += "/channels.yaml";

    YAML::Node channel_config = YAML::LoadFile(config_address);
    enable_channel = channel_config["enable"].as<string>();
    wbc_cmd_channel = channel_config["wbc_cmd"].as<string>();
    wbc_state_channel = channel_config["wbc_state"].as<string>();
    servo_cmd_channel = channel_config["servo_cmd"].as<string>();
    robot_state_channel = channel_config["robot_state"].as<string>();
    servo_state_channel = channel_config["servo_state"].as<string>();
    gait_phase_sig_channel = channel_config["gait_phase"].as<string>();

    enable = false;
    reset = true;

    servo_pos.setZero();
    servo_vel.setZero();
    servo_torq.setZero();
    phase_signal << 1, 1, 1, 1;
    for (int i = 0; i < 3; i++)
    {
        wbcStateMsg.r1_grf[i] = 0.0;
        wbcStateMsg.l1_grf[i] = 0.0;
        wbcStateMsg.r2_grf[i] = 0.0;
        wbcStateMsg.l2_grf[i] = 0.0;
    }

    leg_cmd.r1_pos <<  0.225, -0.1, -0.17;
    leg_cmd.l1_pos <<  0.225,  0.1, -0.17;
    leg_cmd.r2_pos << -0.225, -0.1, -0.17;
    leg_cmd.l2_pos << -0.225,  0.1, -0.17;
    leg_cmd.r1_grf.setZero();
    leg_cmd.l1_grf.setZero();
    leg_cmd.r2_grf.setZero();
    leg_cmd.l2_grf.setZero();
    leg_cmd.r1_vel.setZero();
    leg_cmd.l1_vel.setZero();
    leg_cmd.r2_vel.setZero();
    leg_cmd.l2_vel.setZero();
    leg_cmd.r1_acc.setZero();
    leg_cmd.l1_acc.setZero();
    leg_cmd.r2_acc.setZero();
    leg_cmd.l2_acc.setZero();

    body_state.pos.setZero();
    body_state.orientation.setZero();
    body_state.orientation_quaternion << 0.0, 0.0, 0.0, 1.0;
    body_state.ang_vel.setZero();
    body_state.lin_vel.setZero();

    body_cmd.pos.setZero();
    body_cmd.orientation.setZero();
    body_cmd.orientation_quaternion << 0.0, 0.0, 0.0, 1.0;
    body_cmd.ang_vel.setZero();
    body_cmd.lin_vel.setZero();

    leg_state.r1_grf.setZero();
    leg_state.l1_grf.setZero();
    leg_state.r2_grf.setZero();
    leg_state.l2_grf.setZero();
    leg_state.r1_pos.setZero();
    leg_state.l1_pos.setZero();
    leg_state.r2_pos.setZero();
    leg_state.l2_pos.setZero();
    leg_state.r1_vel.setZero();
    leg_state.l1_vel.setZero();
    leg_state.r2_vel.setZero();
    leg_state.l2_vel.setZero();
    leg_state.r1_acc.setZero();
    leg_state.l1_acc.setZero();
    leg_state.r2_acc.setZero();
    leg_state.l2_acc.setZero();
    leg_state.contacts = std::vector<bool>(4, false);
    // leg_state.contacts = {false, false, false, false};
}

void LCMExchanger::start_exchanger()
{
    thWbcCmd = make_unique<thread> (&LCMExchanger::wbcCmdThread, this);
    thRobotState = make_unique<thread> (&LCMExchanger::robotStateThread, this);
    thServoState = make_unique<thread> (&LCMExchanger::servoStateThread, this);
    thEnable = make_unique<thread> (&LCMExchanger::enableThread, this);
    thGaitPhaseSig = make_unique<thread> (&LCMExchanger::phaseSigThread, this);
}

void LCMExchanger::wbcCmdHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::wbc_cmd_msg* msg)
{
    // cout << "I got wbc_cmd!" << endl;
    for (int i=0; i<3; i++)
    {
        body_cmd.pos(i) = msg->body.position[i];
        body_cmd.orientation(i) = msg->body.orientation_euler[i];
        body_cmd.orientation_quaternion(i) = msg->body.orientation_quaternion[i];
        body_cmd.lin_vel(i) = msg->body.lin_vel[i];
        body_cmd.ang_vel(i) = msg->body.ang_vel[i];

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
    body_cmd.orientation_quaternion(3) = msg->body.orientation_quaternion[3];
}

void LCMExchanger::robotStateHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const mors_msgs::robot_state_msg* msg)
{
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
        body_state.orientation_quaternion(i) = msg->body.orientation_quaternion[i];
        body_state.lin_vel(i) = msg->body.lin_vel[i];
        body_state.ang_vel(i) = msg->body.ang_vel[i];
    }
    leg_state.contacts[3] = msg->legs.contact_states[3];
    body_state.orientation_quaternion(3) = msg->body.orientation_quaternion[3];
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
    enable = msg->leg_controller_en;
    reset = msg->leg_controller_reset;
}

void LCMExchanger::phaseSigHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const mors_msgs::phase_signal_msg* msg)
{
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

void LCMExchanger::robotStateThread()
{
    robot_state_subscriber.subscribe(robot_state_channel, &LCMExchanger::robotStateHandler, this);
    while(true)
    {
        robot_state_subscriber.handle();
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
    gait_phase_sig_subscriber.subscribe(gait_phase_sig_channel, &LCMExchanger::phaseSigHandler, this);
    while(true)
    {
        gait_phase_sig_subscriber.handle();
    }
}


void LCMExchanger::sendServoCmd(const JointVector& position,
                                const JointVector& velocity,
                                const JointVector& torque,
                                const JointVector& kp,
                                const JointVector& kd)
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

void LCMExchanger::sendWbcState(const ForceVector& grf)
{
    for (int i = 0; i < 3; i++)
    {
        wbcStateMsg.r1_grf[i] = grf(PIN_R1 * 3 + i);
        wbcStateMsg.l1_grf[i] = grf(PIN_L1 * 3 + i);
        wbcStateMsg.r2_grf[i] = grf(PIN_R2 * 3 + i);
        wbcStateMsg.l2_grf[i] = grf(PIN_L2 * 3 + i);
    }

    wbc_state_publisher.publish(wbc_state_channel, &wbcStateMsg);
}

void LCMExchanger::getWbcCmdData(RobotData& body_cmd, LegData& leg_cmd)
{
    body_cmd.pos = this->body_cmd.pos;
    body_cmd.orientation = this->body_cmd.orientation;
    body_cmd.orientation_quaternion = this->body_cmd.orientation_quaternion;
    body_cmd.lin_vel = this->body_cmd.lin_vel;
    body_cmd.ang_vel = this->body_cmd.ang_vel;

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

RobotData LCMExchanger::getBodyState()
{
    return body_state;
}

LegData LCMExchanger::getLegState()
{
    return leg_state;
}

void LCMExchanger::getServoStateData(JointVector& position, JointVector& velocity, JointVector& torque)
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

LCMExchanger::PhaseVector LCMExchanger::getGaitPhaseSignal() const
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
