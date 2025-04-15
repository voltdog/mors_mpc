#include "lcm_data_exchange_sc.hpp"
#include <unistd.h>

// #define KT 0.74
// #define GEAR_RATIO 10.0

LCMExchanger::LCMExchanger()
{
    
    if(!robot_cmd_subscriber.good())
        return;
    if(!robot_state_subscriber.good())
        return;
    if(!phase_signal_subscriber.good())
        return;

    string config_address = mors_sys::GetEnv("CONFIGPATH");//cwd;
    config_address += "/channels.yaml";

    YAML::Node channel_config = YAML::LoadFile(config_address);//"/home/user/mors_mpc_control/config/channels.yaml");
    robot_cmd_channel = channel_config["robot_cmd"].as<string>();
    robot_state_channel = channel_config["robot_state"].as<string>();
    phase_signal_channel = channel_config["gait_phase"].as<string>();
    grf_cmd_channel = channel_config["grf_cmd"].as<string>();

    // cmd_vel.resize(6);
    // cmd_pose.resize(6);
    phase.resize(4);
    robot_state.ang_vel.resize(3);
    robot_state.lin_vel.resize(3);
    robot_state.orientation.resize(3);
    robot_state.pos.resize(3);
    robot_cmd.ang_vel.resize(3);
    robot_cmd.lin_vel.resize(3);
    robot_cmd.orientation.resize(3);
    robot_cmd.pos.resize(3);
    leg_state.contacts.resize(4);
    leg_state.r1_pos.resize(3);
    leg_state.l1_pos.resize(3);
    leg_state.r2_pos.resize(3);
    leg_state.l2_pos.resize(3);

    // cmd_vel.setZero();
    // cmd_pose.setZero();
    phase << 1,1,1,1;
    robot_state.ang_vel.setZero();
    robot_state.lin_vel.setZero();
    robot_state.orientation.setZero();
    robot_state.pos.setZero();
    robot_cmd.ang_vel.setZero();
    robot_cmd.lin_vel.setZero();
    robot_cmd.orientation.setZero();
    robot_cmd.pos.setZero();
    leg_state.contacts.setZero();
    leg_state.r1_pos.setZero();
    leg_state.l1_pos.setZero();
    leg_state.r2_pos.setZero();
    leg_state.l2_pos.setZero();
}

void LCMExchanger::start_exchanger()
{
    thRobotCmd = make_unique<thread> (&LCMExchanger::robotCmdThread, this);
    thRobotState = make_unique<thread> (&LCMExchanger::robotStateThread, this);
    thPhaseSignal = make_unique<thread> (&LCMExchanger::phaseSignalThread, this);
}

void LCMExchanger::robotCmdHandler(const lcm::ReceiveBuffer* rbuf,
                                    const std::string& chan,
                                    const mors_msgs::robot_cmd_msg* msg)
{
    for (int i=0; i<3; i++)
    {
        // cmd_pose(i) = msg->cmd_pose[i];
        // cmd_vel(i) = msg->cmd_vel[i];
        robot_cmd.pos(i) = msg->cmd_pose[i];
        robot_cmd.orientation(i) = msg->cmd_pose[i+3];
        robot_cmd.lin_vel(i) = msg->cmd_vel[i];
        robot_cmd.ang_vel(i) = msg->cmd_vel[i+3];
    }
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

        leg_state.contacts(i) = msg->legs.contact_states[i];

        robot_state.pos(i) = msg->body.position[i];
        robot_state.orientation(i) = msg->body.orientation[i];
        robot_state.lin_vel(i) = msg->body.lin_vel[i];
        robot_state.ang_vel(i) = msg->body.ang_vel[i];
    }
    leg_state.contacts(3) = msg->legs.contact_states[3];
}

void LCMExchanger::phaseSignalHandler(const lcm::ReceiveBuffer* rbuf,
                                    const std::string& chan,
                                    const mors_msgs::phase_signal_msg* msg)
{
    for (int i=0; i<4; i++)
        phase(i) = msg->phase[i];
    // cout << phase.transpose() << endl;
}

void LCMExchanger::robotCmdThread()
{   
    robot_cmd_subscriber.subscribe(robot_cmd_channel, &LCMExchanger::robotCmdHandler, this);
    while(true)
    {
        robot_cmd_subscriber.handle();
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
        // cout << "grf thread" << endl;
        // sleep(0.001);
    }
}

void LCMExchanger::phaseSignalThread()
{
    phase_signal_subscriber.subscribe(phase_signal_channel, &LCMExchanger::phaseSignalHandler, this);
    while(true)
    {
        phase_signal_subscriber.handle();
        // cout << "imu thread" << endl;
        // sleep(0.001);
    }
}


void LCMExchanger::sendGrfCmd(LegData &leg_data)
{
    for (int i=0; i<3; i++)
    {
        grfCmdMsg.r1_grf[i] = leg_data.r1_grf(i);
        grfCmdMsg.l1_grf[i] = leg_data.l1_grf(i);
        grfCmdMsg.r2_grf[i] = leg_data.r2_grf(i);
        grfCmdMsg.l2_grf[i] = leg_data.l2_grf(i);
    }
    grf_cmd_publisher.publish(grf_cmd_channel, &grfCmdMsg);
}

RobotData LCMExchanger::getRobotCmd()
{
    return robot_cmd;
}

RobotData LCMExchanger::getRobotState()
{
    return robot_state;
}

LegData LCMExchanger::getLegsState()
{
    return leg_state;
}

VectorXd LCMExchanger::getPhaseSignals()
{
    return phase;
}



LCMExchanger::~LCMExchanger()
{

}
