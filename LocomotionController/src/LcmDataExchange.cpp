#include "LcmDataExchange.hpp"
#include <unistd.h>

// #define KT 0.74
// #define GEAR_RATIO 10.0

LCMExchanger::LCMExchanger()
{
    
    if(!robot_cmd_subscriber.good())
        return;
    if(!robot_state_subscriber.good())
        return;
    if(!enable_subscriber.good())
        return;
    if(!gait_params_subscriber.good()) 
        return;

    string config_address = mors_sys::GetEnv("CONFIGPATH");//cwd;
    config_address += "/channels.yaml";

    YAML::Node channel_config = YAML::LoadFile(config_address);//"/home/user/mors_mpc_control/config/channels.yaml");
    enable_channel = channel_config["enable"].as<string>();
    robot_cmd_channel = channel_config["robot_cmd"].as<string>();
    mpc_cmd_channel = channel_config["mpc_cmd"].as<string>();
    wbc_cmd_channel = channel_config["wbc_cmd"].as<string>();
    robot_state_channel = channel_config["robot_state"].as<string>();    
    gait_params_channel = channel_config["gait_params"].as<string>();
    phase_signal_channel = channel_config["gait_phase"].as<string>();

    leg_state.contacts.resize(4);
    leg_state.contacts = {false, false, false, false};

    t_sw = 0.2;
    t_st = 0.3;
    gait_type = {0.0, 0, 0, 0.0};
    standing = true;
    stride_height = 0.06;
    active_legs = {true, true, true, true};
    adaptation_type = 0;

    enable = false;
}

void LCMExchanger::start_exchanger()
{
    thRobotCmd = make_unique<thread> (&LCMExchanger::robotCmdThread, this);
    thRobotState = make_unique<thread> (&LCMExchanger::robotStateThread, this);
    thEnable = make_unique<thread> (&LCMExchanger::enableThread, this);
    thGaitParams = make_unique<thread> (&LCMExchanger::gaitParamsThread, this);
}

void LCMExchanger::robotCmdHandler(const lcm::ReceiveBuffer* rbuf,
                                    const std::string& chan,
                                    const mors_msgs::robot_cmd_msg* msg)
{
    for (int i=0; i<3; i++)
    {
        robot_cmd.pos(i) = msg->cmd_pose[i];
        robot_cmd.orientation(i) = msg->cmd_pose[i+3];
        robot_cmd.lin_vel(i) = msg->cmd_vel[i];
        robot_cmd.ang_vel(i) = msg->cmd_vel[i+3];
    }
    active_legs.assign(msg->active_legs, msg->active_legs+4);
    adaptation_type = msg->adaptation_type;
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

        // leg_state.r1_vel(i) = msg->legs.r1_vel[i];
        // leg_state.l1_vel(i) = msg->legs.l1_vel[i];
        // leg_state.r2_vel(i) = msg->legs.r2_vel[i];
        // leg_state.l2_vel(i) = msg->legs.l2_vel[i];

        // leg_state.r1_grf(i) = msg->legs.r1_grf[i];
        // leg_state.l1_grf(i) = msg->legs.l1_grf[i];
        // leg_state.r2_grf(i) = msg->legs.r2_grf[i];
        // leg_state.l2_grf(i) = msg->legs.l2_grf[i];

        leg_state.contacts[i] = msg->legs.contact_states[i];

        robot_state.pos(i) = msg->body.position[i];
        robot_state.orientation(i) = msg->body.orientation[i];
        robot_state.lin_vel(i) = msg->body.lin_vel[i];
        robot_state.ang_vel(i) = msg->body.ang_vel[i];
    }
    leg_state.contacts[3] = msg->legs.contact_states[3];
}

void LCMExchanger::gaitParamsHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const mors_msgs::gait_params_msg* msg)
{
    t_sw = msg->t_sw;
    t_st = msg->t_st;
    standing = msg->standing;
    stride_height = msg->stride_height;
    gait_type.assign(msg->gait_type, msg->gait_type+4);
}

void LCMExchanger::enableHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::enable_msg* msg)
{
    enable = msg->locomotion_en;
}

void LCMExchanger::robotCmdThread()
{   
    robot_cmd_subscriber.subscribe(robot_cmd_channel, &LCMExchanger::robotCmdHandler, this);
    while(true)
    {
        robot_cmd_subscriber.handle();
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

void LCMExchanger::gaitParamsThread()
{
    gait_params_subscriber.subscribe(gait_params_channel, &LCMExchanger::gaitParamsHandler, this);
    while(true)
    {
        gait_params_subscriber.handle();
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

RobotData LCMExchanger::getRobotCmd()
{
    return robot_cmd;
}

RobotData LCMExchanger::getBodyState()
{
    return robot_state;
}

LegData LCMExchanger::getLegState()
{
    return leg_state;
}

void LCMExchanger::get_gait_params(double& t_st, double& t_sw, vector<double>& gait_type, bool& standing, double& stride_height)
{
    t_st = this->t_st;
    t_sw = this->t_sw;
    gait_type = this->gait_type;
    standing = this->standing;
    stride_height= this->stride_height;
}
 
bool LCMExchanger::get_enable()
{
    return enable;
}

void LCMExchanger::get_active_legs(vector<bool>& active_legs)
{
    active_legs = this->active_legs;
}

int LCMExchanger::get_adaptation_type()
{
    return adaptation_type;
}

void LCMExchanger::sendWbcCmd(RobotData& robot_data, LegData& leg_data)
{
    for (int i=0; i<3; i++)
    {
        wbcCmdMsg.body.position[i] = robot_data.pos(i);
        wbcCmdMsg.body.orientation_euler[i] = robot_data.orientation(i);
        wbcCmdMsg.body.lin_vel[i] = robot_data.lin_vel(i);
        wbcCmdMsg.body.ang_vel[i] = robot_data.ang_vel(i);

        wbcCmdMsg.legs.r1_pos[i] = leg_data.r1_pos(i);
        wbcCmdMsg.legs.l1_pos[i] = leg_data.l1_pos(i);
        wbcCmdMsg.legs.r2_pos[i] = leg_data.r2_pos(i);
        wbcCmdMsg.legs.l2_pos[i] = leg_data.l2_pos(i);

        wbcCmdMsg.legs.r1_vel[i] = leg_data.r1_vel(i);
        wbcCmdMsg.legs.l1_vel[i] = leg_data.l1_vel(i);
        wbcCmdMsg.legs.r2_vel[i] = leg_data.r2_vel(i);
        wbcCmdMsg.legs.l2_vel[i] = leg_data.l2_vel(i);

        wbcCmdMsg.legs.r1_grf[i] = leg_data.r1_grf(i);
        wbcCmdMsg.legs.l1_grf[i] = leg_data.l1_grf(i);
        wbcCmdMsg.legs.r2_grf[i] = leg_data.r2_grf(i);
        wbcCmdMsg.legs.l2_grf[i] = leg_data.l2_grf(i);
    }

    wbc_cmd_publisher.publish(wbc_cmd_channel, &wbcCmdMsg);
}

void LCMExchanger::sendPhaseSig(vector<int>& phase, vector<double>& phi, double t)
{
    for (int i = 0; i < 4; i++)
    {
        phaseSigMsg.phase[i] = phase[i];
        phaseSigMsg.phi[i] = phi[i];
    }
    phaseSigMsg.t = t;
    phase_sig_publisher.publish(phase_signal_channel, &phaseSigMsg);
}

void LCMExchanger::sendMpcCmd(RobotData& robot_cmd, vector<bool> &active_legs)
{
    for (int i = 0; i < 3; i++)
    {
        mpcCmdMsg.cmd_vel[i] = robot_cmd.lin_vel[i];
        mpcCmdMsg.cmd_vel[i+3] = robot_cmd.ang_vel[i];
        mpcCmdMsg.cmd_pose[i] = robot_cmd.pos[i];
        mpcCmdMsg.cmd_pose[i+3] = robot_cmd.orientation[i];

        mpcCmdMsg.active_legs[i] = active_legs[i];
    }
    mpcCmdMsg.active_legs[3] = active_legs[3];

    robot_ref_publisher.publish(mpc_cmd_channel, &mpcCmdMsg);
}

LCMExchanger::~LCMExchanger()
{

}
