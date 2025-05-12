#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include "lcm_data_exchange_ml.hpp"
#include "structs.hpp"
#include "csv_maintaner.hpp"
#include <vbmath.hpp>
#include <yaml-cpp/yaml.h>
#include "system_functions.hpp"

using namespace std;
using namespace Eigen;
using namespace YAML;
using namespace std::chrono;

// current time
auto now() 
{
  return std::chrono::steady_clock::now(); 
}

int main() {
    cout << "MORS Logger starting..." << endl;
    // load config
    string config_address = mors_sys::GetEnv("CONFIGPATH");

    string dt_config_address = config_address + "/timesteps.yaml";
    YAML::Node dt_config = YAML::LoadFile(dt_config_address);
    double module_dt = dt_config["mors_logger_dt"].as<double>(); 

    auto dt = std::chrono::duration<double>(module_dt);//1ms;

    // define lcm exchanger
    LCMExchanger lcmExch;
    lcmExch.start_exchanger();
    std::this_thread::sleep_for(5ms);

    // define lcm data
    LegData leg_cmd;
    ImuData imu_data;
    ServoData servo_state;
    ServoData servo_cmd;
    Odometry odometry;
    ServoData servo_state_filt;
    LegData leg_state;
    RobotData body_state;
    RobotData body_cmd;
    Vector4i phase;
    Vector4d phi;

    int control_type = 0;
    bool leg_controller_enable, leg_controller_reset;
    bool locomotion_enable, locomotion_reset;
    bool action_ctr_enable, action_ctr_reset;

    // define csv
    CSVMaintainer csv;
    csv.init();

    cout << "MORS Logger started" << endl;

    double t = 0.0;

    while(true)
    {
        // Calculating current time
        const auto start{ now() };

        // Put your code here
        // -----------------------------------------------
        
        leg_cmd = lcmExch.getLegCmdData();
        imu_data = lcmExch.getImuData();
        servo_state = lcmExch.getServoStateData();
        servo_cmd = lcmExch.getServoCmdData();
        lcmExch.getEnableData(leg_controller_enable, leg_controller_reset,
                                locomotion_enable, locomotion_reset,
                                action_ctr_enable, action_ctr_reset);
        control_type = lcmExch.getCtrlTypeData();
        odometry = lcmExch.getOdometry();
        servo_state_filt = lcmExch.getServoStateFiltData();
        body_state = lcmExch.getBodyState();
        body_cmd = lcmExch.getRobotCmd();
        leg_state = lcmExch.getLegState();
        lcmExch.getPhaseSig(phase, phi);

        // save arrays
        csv.write_servo_state(t, servo_state);
        csv.write_servo_state_filt(t, servo_state_filt);
        csv.write_servo_cmd(t, servo_cmd);
        csv.write_imu_data(t, imu_data);
        csv.write_leg_cmd(t, leg_cmd);
        csv.write_odometry(t, odometry);
        csv.write_control_type(t, control_type);
        csv.write_enable(t, leg_controller_enable, leg_controller_reset,
                            locomotion_enable, locomotion_reset,
                            action_ctr_enable, action_ctr_reset);
        
        csv.write_robot_state(t, body_state, leg_state);
        csv.write_robot_cmd(t, body_cmd);
        csv.write_phase_sig(t, phase, phi);
        
        t += module_dt;
        // Wait until spinning time
        while(true)
        {
            std::chrono::duration<double, std::milli> elapsed{now() - start};
            if (elapsed >= dt)
            {
                // cout << "Waited for : " << elapsed.count() << " ms" << endl;
                break;
            }
        }
    }

    return 0;
}