#include "VisionConfig.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <stdexcept>

namespace mors::fsm {

VisionConfig::VisionConfig(const std::filesystem::path& directory) {
    const auto config = YAML::LoadFile((directory / "locomotion_controller.yaml").string());
    const auto body = YAML::LoadFile((directory / "robot.yaml").string())["robot_config"];
    const auto periods = YAML::LoadFile((directory / "timesteps.yaml").string());
    const auto mpc = config["stance_controller_mpc"];
    const auto wbc = config["wbic"];
    const auto gait = config["gait_scheduler"];
    const auto reference = config["reference_generator"];
    const auto swing = config["swing_controller"];
    auto positive = [](const YAML::Node& section, const char* key) {
        double value = section[key].as<double>();
        if (!std::isfinite(value) || value <= 0) throw std::invalid_argument(key);
        return value;
    };
    touchdown_phase = gait["start_td_detecting"].as<double>();
    contact_velocity = gait["vz_contact_thresh"].as<double>(0.1);
    contact_debounce = gait["contact_debounce"].as<int>(3);
    transition_s = positive(gait, "delta_T_gt");
    filter_hz = positive(reference, "c_freq");
    zero_velocity = reference["zero_vel_thresh"].as<double>();
    foot_radius = positive(reference, "foot_valid_radius");
    interleave_x = swing["interleave_x"].as<std::array<double, 4>>();
    interleave_y = swing["interleave_y"].as<std::array<double, 4>>();
    near_ground_velocity = swing["dz_near_ground"].as<double>();
    rising_proportion = swing["rising_proportion"].as<double>();
    footstep_gain = swing["k1_fsp"].as<double>();
    dt = positive(periods, "locomotion_controller");
    mpc_dt = positive(periods, "stance_controller_dt");
    wbic.timestep = positive(periods, "wbic_controller_dt");
    handoff_s = positive(wbc, "handoff_timeout_ms") / 1000;
    recovery_stance = positive(config["recovery"], "t_st");
    recovery_swing = positive(config["recovery"], "t_sw");
    const auto frame = config["vel_cmd_frame"].as<std::string>("local");
    if (frame != "local" && frame != "global") throw std::invalid_argument("vel_cmd_frame");
    local_velocity = frame == "local";
    horizon = mpc["horizon"].as<int>();
    friction = mpc["friction"].as<double>();
    f_min = mpc["f_min"].as<double>();
    f_max = mpc["f_max"].as<double>();
    const auto weights = [&](const char* key, const char* gain, int size) -> Eigen::VectorXd {
        auto values = mpc[key].as<std::vector<double>>();
        if (static_cast<int>(values.size()) != size) throw std::invalid_argument(key);
        return Eigen::Map<const Eigen::VectorXd>(values.data(), size) * mpc[gain].as<double>();
    };
    Q = weights("Q", "Q_gain", 13);
    R = weights("R", "R_gain", 12);
    robot.bx = body["bx"].as<double>();
    robot.by = body["by"].as<double>();
    robot.l1 = body["l1"].as<double>();
    robot.l2 = body["l2"].as<double>();
    robot.l3 = body["l3"].as<double>();
    robot.d1 = body["d1"].as<double>();
    robot.d2 = body["d2"].as<double>();
    robot.d3 = body["d3"].as<double>();
    robot.m1 = body["m1"].as<double>();
    robot.m2 = body["m2"].as<double>();
    robot.m3 = body["m3"].as<double>();
    robot.g = body["g"].as<double>();
    robot.kt = body["kt"].as<double>();
    robot.gear_ratio = body["gear_ratio"].as<double>();
    robot.l_cz_2 = body["Pc2"][2].as<double>();
    robot.l_cx_3 = body["Pc3"][0].as<double>();
    robot.M_b = body["M"].as<double>();
    robot.I_b.resize(3, 3);
    for (int i = 0; i < 3; ++i) {
        robot.joint_tau_max_array[i] = body["tau_max"][i].as<double>();
        robot.joint_vel_max_array[i] = body["vel_max"][i].as<double>();
        for (int j = 0; j < 3; ++j) robot.I_b(i,j) = body["I_b"][i][j].as<double>();
    }
    wbic.ground_fric = friction;
    wbic.fz_min = f_min;
    wbic.fz_max = f_max;
    wbic.debug_mode = body["debug_mode"].as<bool>(false);
    wbic.Qf_entry = wbc["Qf_entry"].as<double>();
    wbic.Qa_entry = wbc["Qa_entry"].as<double>();
    wbic.body_ori_task_kp = wbc["body_ori_task_kp"].as<double>();
    wbic.body_ori_task_kd = wbc["body_ori_task_kd"].as<double>();
    wbic.body_pos_task_kp = wbc["body_pos_task_kp"].as<double>();
    wbic.body_pos_task_kd = wbc["body_pos_task_kd"].as<double>();
    wbic.tip_pos_task_kp = wbc["tip_pos_task_kp"].as<double>();
    wbic.tip_pos_task_kd = wbc["tip_pos_task_kd"].as<double>();
    wbic.joint_kp_stance = wbc["joint_kp_stance"].as<double>();
    wbic.joint_kd_stance = wbc["joint_kd_stance"].as<double>();
    wbic.joint_kp_swing = wbc["joint_kp_swing"].as<double>();
    wbic.joint_kd_swing = wbc["joint_kd_swing"].as<double>();
    wbic.realtime_priority = wbc["realtime_priority"].as<int>(wbic.realtime_priority);
    wbic.cpu_index = wbc["cpu_index"].as<int>(wbic.cpu_index);
    wbic.spin_guard_us = wbc["spin_guard_us"].as<int>(wbic.spin_guard_us);
}

} // namespace mors::fsm
