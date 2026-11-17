#include "LocomotionNode.hpp"

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>

namespace mors::fsm {
namespace {

using ModeCommand = mors_msgs::robot_mode_cmd_msg;
using ModeResult = mors_msgs::robot_mode_result_msg;
using ModeState = mors_msgs::robot_mode_state_msg;
using Clock = std::chrono::steady_clock;

std::filesystem::path config_directory() 
{
    if (const char* path = std::getenv("CONFIGPATH"); path && *path) 
    {
        return path;
    }
    return FSM_DEFAULT_CONFIG_DIR;
}

Clock::duration load_period(const std::filesystem::path& config_path) 
{
    const auto config = YAML::LoadFile((config_path / "timesteps.yaml").string());
    const double dt = config["fsm_dt"].as<double>();
    const double max_dt = std::chrono::duration<double>(Clock::time_point::max() - Clock::now()).count();

    if (!std::isfinite(dt) || dt <= 0.0 || dt >= max_dt) 
        throw std::invalid_argument("Invalid fsm_dt");
    
    const auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(dt));
    
    if (period <= Clock::duration::zero()) 
        throw std::invalid_argument("fsm_dt is below steady_clock resolution");

    return period;
}

SafetyMonitor load_safety_monitor(const YAML::Node& config)
{
    // ROBOT_STATE сохраняет порядок SERVO_STATE, по три сустава на каждую ногу.
    constexpr std::array joint_names{
        "abad_joint_R1", "hip_joint_R1", "knee_joint_R1",
        "abad_joint_L1", "hip_joint_L1", "knee_joint_L1",
        "abad_joint_R2", "hip_joint_R2", "knee_joint_R2",
        "abad_joint_L2", "hip_joint_L2", "knee_joint_L2",
    };
    static_assert(joint_names.size() == mors::wbic::kJointDofs);

    const auto emergency = config["emergency"];
    const auto minimum = emergency["min_angles"].as<std::vector<double>>();
    const auto maximum = emergency["max_angles"].as<std::vector<double>>();

    if (minimum.size() != joint_names.size() || maximum.size() != joint_names.size())
        throw std::invalid_argument("emergency.min_angles and max_angles must contain 12 limits");

    JointLimitList limits;
    limits.reserve(joint_names.size());
    for (std::size_t i = 0; i < joint_names.size(); ++i)
        limits.emplace_back(joint_names[i], JointLimits(minimum[i], maximum[i]));
    
    return SafetyMonitor(std::move(limits), emergency["robot_state_timeout_ms"].as<double>() / 1000.0);
}

double read_setting(const YAML::Node& section, const char* key, double fallback) {
    if (section) {
        if (const auto value = section[key]) {
            return value.as<double>();
        }
    }
    return fallback;
}

LocomotionTransitionConfig load_transition_config(const YAML::Node& section) {
    if (section && !section.IsMap()) {
        throw std::invalid_argument("fsm_locomotion must be a mapping");
    }
    LocomotionTransitionConfig config;
    config.command_timeout_s = read_setting(section, "command_timeout_s", config.command_timeout_s);
    config.start_linear_mps = read_setting(section, "start_linear_mps", config.start_linear_mps);
    config.start_yaw_radps = read_setting(section, "start_yaw_radps", config.start_yaw_radps);
    config.stop_linear_mps = read_setting(section, "stop_linear_mps", config.stop_linear_mps);
    config.stop_yaw_radps = read_setting(section, "stop_yaw_radps", config.stop_yaw_radps);
    config.stop_hold_s = read_setting(section, "stop_hold_s", config.stop_hold_s);

    const auto load_pair = [&](const char* key, LocomotionTransitionConfig::RecoveryThresholds defaults) {
        if (!section) {
            return defaults;
        }
        const auto pair = section[key];
        return LocomotionTransitionConfig::RecoveryThresholds{
            read_setting(pair, "position_m", defaults.position_m),
            read_setting(pair, "velocity_mps", defaults.velocity_mps),
        };
    };
    config.recovery_entry = load_pair("recovery_entry", config.recovery_entry);
    config.recovery_exit_walking = load_pair("recovery_exit_walking", config.recovery_exit_walking);
    config.recovery_exit_standing = load_pair("recovery_exit_standing", config.recovery_exit_standing);
    return config;
}

GaitStopConfig load_gait_stop_config(const YAML::Node& section) {
    GaitStopConfig config;
    if (section) {
        const auto stop = section["gait_stop"];
        config.linear_mps = read_setting(stop, "linear_mps", config.linear_mps);
        config.yaw_radps = read_setting(stop, "yaw_radps", config.yaw_radps);
        config.hold_s = read_setting(stop, "hold_s", config.hold_s);
    }
    return config;
}

std::optional<StateID> mode_target(std::int8_t target) 
{
    switch (target) 
    {
        case ModeCommand::IDLE: return StateID::IDLE; // PASSIVE на проводе.
        case ModeCommand::STANDING: return StateID::STANDING;
        case ModeCommand::LOCOMOTION: return StateID::LOCOMOTION;
        default: return std::nullopt;
    }
}

std::int8_t mode_code(StateID state) 
{
    switch (state) 
    {
        case StateID::IDLE: return ModeState::PASSIVE;
        case StateID::STANDUP: return ModeState::STANDUP;
        case StateID::STANDING: return ModeState::STANDING;
        case StateID::LOCOMOTION: return ModeState::LOCOMOTION;
        case StateID::LAYDOWN: return ModeState::LAYDOWN;
        case StateID::ACTION: return ModeState::ACTION;
        case StateID::EXTERNAL: return ModeState::EXTERNAL;
        case StateID::FAILURE: return ModeState::FAILURE;
    }
    throw std::logic_error("Invalid FSM state");
}

std::string fault_text(const std::optional<SafetyFault>& fault) {
    return fault ? fault->code + ": " + fault->description : std::string{};
}

} // namespace

LocomotionNode::LocomotionNode() : LocomotionNode(config_directory()) {}

LocomotionNode::LocomotionNode(const std::filesystem::path& config_path)
    : LocomotionNode(config_path, YAML::LoadFile((config_path / "robot.yaml").string())) {}

LocomotionNode::LocomotionNode(const std::filesystem::path& config_path, const YAML::Node& robot_config)
    : period_(load_period(config_path)),
      safety_monitor_(load_safety_monitor(robot_config)),
      transition_monitor_(load_transition_config(robot_config["fsm_locomotion"])),
      fsm_(safety_monitor_, &status_, &controller_, &readiness_checker_,
           load_gait_stop_config(robot_config["fsm_locomotion"])),
      exchanger_(config_path) 
{
    exchanger_.start_exchanger();
    controller_.start_thread();
}

LocomotionNode::~LocomotionNode() {
    exchanger_.stop_exchanger();
    controller_.stop_thread();
}

std::chrono::steady_clock::duration LocomotionNode::period() const 
{
    return period_;
}

void LocomotionNode::update_status(const std::optional<RobotStateSnapshot>& observation, double now_s)
{
    status_.safety = safety_monitor_.evaluate(observation, now_s);
    status_.ready_to_move = readiness_checker_.is_ready(status_, now_s);
    if (!observation)
        return;

    // Оценщик публикует линейную скорость в мире, а угловую уже в системе корпуса.
    const auto& body = observation->body;
    const auto& q = body.orientation_quaternion;
    const Eigen::Quaterniond world_from_body(q[3], q[0], q[1], q[2]);
    const double norm = world_from_body.norm();
    if (std::isfinite(norm) && norm > 1e-6) 
    {
        const Eigen::Vector3d velocity = world_from_body.normalized().conjugate() * body.lin_vel;
        status_.vx = velocity.x();
        status_.vy = velocity.y();
    } 
    else 
    {
        // Некорректная ориентация не должна превратить измеренную скорость в ноль.
        status_.vx = status_.vy = std::numeric_limits<double>::quiet_NaN();
    }
    status_.yaw_rate = body.ang_vel.z();
    std::copy_n(observation->legs.contacts.begin(), status_.contacts.size(), status_.contacts.begin());
}

void LocomotionNode::request_mode(std::int8_t target_code, bool fault_cleared) {
    const auto target = mode_target(target_code);
    if (!target) {
        exchanger_.sendRobotModeResult(ModeCommand::SET_MODE, ModeResult::REJECTED, "Unsupported mode target");
    } else if (fault_cleared) {
        exchanger_.sendRobotModeResult(ModeCommand::SET_MODE, ModeResult::REJECTED, "Send a new SET_MODE after CLEAR_FAULT");
    } else if (fsm_.state_id == StateID::FAILURE) {
        exchanger_.sendRobotModeResult(ModeCommand::SET_MODE, ModeResult::REJECTED, "Cannot change mode in FAILURE");
    } else {
        if (pending_mode_request_) {
            exchanger_.sendRobotModeResult(ModeCommand::SET_MODE, ModeResult::CANCELED, "Superseded by a new SET_MODE");
        }
        operator_request_ = target;
        pending_mode_request_ = target;
        exchanger_.sendRobotModeResult(ModeCommand::SET_MODE, ModeResult::ACCEPTED, "");
    }
}

void LocomotionNode::finish_mode_request() {
    const auto state = fsm_.state_id;
    if (state == StateID::FAILURE) {
        operator_request_.reset();
    }
    if (!pending_mode_request_) {
        return;
    }
    if (state == StateID::FAILURE) {
        exchanger_.sendRobotModeResult(ModeCommand::SET_MODE, ModeResult::FAILED,
                                       fault_text(safety_monitor_.fault()));
    } else {
        // Восстановление тоже имеет StateID::LOCOMOTION, но не подтверждает разрешение ходьбы.
        const auto requested = *pending_mode_request_;
        const bool reached = requested == StateID::LOCOMOTION
            ? state == StateID::STANDING || (state == StateID::LOCOMOTION && locomotion_mode_ == LocomotionMode::WALKING)
            : state == requested;
        if (!reached) {
            return;
        }
        exchanger_.sendRobotModeResult(ModeCommand::SET_MODE, ModeResult::SUCCEEDED, "");
    }
    pending_mode_request_.reset();
}

void LocomotionNode::process_mode_commands(const std::vector<ModeCommand>& commands) 
{
    const bool had_fault = (fsm_.state_id == StateID::FAILURE);
    fsm_.enforce_safety();
    const bool estop = std::ranges::any_of(commands, [](const ModeCommand& command) {
        return command.command == ModeCommand::ESTOP;
    });
    if (estop)
        fsm_.handle_event(Event::ESTOP);

    const bool failed = (fsm_.state_id == StateID::FAILURE);
    if (failed)
        finish_mode_request();

    bool fault_cleared = false;
    for (const auto& command : commands) 
    {
        switch (command.command) 
        {
            case ModeCommand::SET_MODE:
                request_mode(command.target, fault_cleared);
                break;
            case ModeCommand::ESTOP:
                exchanger_.sendRobotModeResult(command.command, ModeResult::SUCCEEDED, "");
                break;
            case ModeCommand::CLEAR_FAULT:
                if (estop || (failed && !had_fault)) 
                {
                    exchanger_.sendRobotModeResult(command.command, ModeResult::REJECTED, "New fault or ESTOP in this tick");
                } 
                else if (const auto reason = fsm_.try_clear_fault()) 
                {
                    exchanger_.sendRobotModeResult(command.command, ModeResult::REJECTED, *reason);
                } 
                else 
                {
                    fault_cleared = true;
                    exchanger_.sendRobotModeResult(command.command, ModeResult::SUCCEEDED, "");
                }
                break;
            case ModeCommand::START_ACTION:
            case ModeCommand::CANCEL_ACTION:
            case ModeCommand::SET_ALGORITHM:
            case ModeCommand::SET_EXTERNAL:
                exchanger_.sendRobotModeResult(command.command, ModeResult::REJECTED, "Unsupported command");
                break;
            default:
                exchanger_.sendRobotModeResult(command.command, ModeResult::REJECTED, "Unknown command");
                break;
        }
    }
}

void LocomotionNode::select_effective_mode(LocomotionSetpoints& setpoints, const RobotData* estimate, double now_s) {
    const auto state = fsm_.state_id;
    // ESTOP и проверки телеметрии уже обработаны, поэтому автоматика не переопределяет аварию.
    if (state == StateID::FAILURE) {
        return;
    }
    if (!transition_monitor_.velocity_is_fresh(setpoints, now_s)) {
        setpoints.robot_cmd.lin_vel.setZero();
        setpoints.robot_cmd.ang_vel.setZero();
    }

    const bool walking_allowed = operator_request_ == StateID::LOCOMOTION;
    auto target = operator_request_;
    if (walking_allowed) {
        target = StateID::STANDING;
    }
    if ((state == StateID::STANDING && target != StateID::IDLE) || state == StateID::LOCOMOTION) {
        const bool recovering = state == StateID::LOCOMOTION && locomotion_mode_ == LocomotionMode::RECOVERY;
        const auto recommended = transition_monitor_.recommend(state, walking_allowed, recovering,
                                                                estimate, setpoints.robot_cmd, now_s);
        if (recommended != LocomotionMode::STANDING) {
            target = StateID::LOCOMOTION;
            locomotion_mode_ = recommended;
        }
        // Во время остановки походки сохраняем текущий режим вплоть до выхода из LOCOMOTION.
        setpoints.mode = locomotion_mode_;
    }
    if (target && target != fsm_.target_state) {
        fsm_.request_state(*target);
    }
}

void LocomotionNode::step() {
    const auto previous_state = fsm_.state_id;
    const auto mode_commands = exchanger_.take_mode_commands();
    const auto observation = exchanger_.get_robot_state();
    auto setpoints = exchanger_.get_setpoints();
    const double now_s = monotonic_seconds();

    update_status(observation, now_s);
    process_mode_commands(mode_commands);

    const bool telemetry_valid = observation && !status_.safety.telemetry_fault;
    const auto* estimate = telemetry_valid && !status_.safety.joint_fault ? &observation->body : nullptr;
    if (telemetry_valid) {
        controller_.set_robot_state(observation->body, observation->legs);
    }

    select_effective_mode(setpoints, estimate, now_s);
    fsm_.update(setpoints);
    if (fsm_.state_id != previous_state) {
        transition_monitor_.on_state_entered(fsm_.state_id, estimate);
        if (fsm_.state_id != StateID::LOCOMOTION) {
            locomotion_mode_ = LocomotionMode::WALKING;
        }
    }
    finish_mode_request();

    const auto& fault = safety_monitor_.fault();
    exchanger_.sendRobotModeState(mode_code(fsm_.state_id),
                                  mode_code(fsm_.target_state.value_or(fsm_.state_id)),
                                  -1, -1, -1, fault.has_value(), fault_text(fault));
}

} // namespace mors::fsm
