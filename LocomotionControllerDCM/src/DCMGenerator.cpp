#include "DCMGenerator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

DCMGenerator::DCMGenerator() : dt(0.005), delta_z(0.2), g(9.81) {

    r_f.resize(PREVIEW_STRIDES_HORIZON);

}

DCMGenerator::~DCMGenerator() {

}

void DCMGenerator::generate(const FootPlanData& foot_plan,
                            const RobotData& robot_state,
                            const RobotData& robot_cmd,
                            std::vector<Eigen::Vector3d>& x_com_traj_out,
                            std::vector<Eigen::Vector3d>& d_x_com_traj_out,
                            std::vector<Eigen::Vector3d>& dd_x_com_traj_out) {
    if (dt <= 0.0) {
        throw std::runtime_error("generate expects positive dt");
    }

    find_r_f(foot_plan, robot_state, r_f);

    double dcm_height = robot_cmd.pos(Z);
    if (!std::isfinite(dcm_height) || dcm_height <= 0.0) {
        dcm_height = robot_state.pos(Z) - r_f.front()(Z);
    }
    if (!std::isfinite(dcm_height) || dcm_height <= 0.0) {
        dcm_height = delta_z;
    }
    if (!std::isfinite(dcm_height) || dcm_height <= 0.0) {
        throw std::runtime_error("generate expects positive finite DCM height");
    }

    delta_z = dcm_height;
    omega = std::sqrt(g / delta_z);

    std::vector<double> t_step;
    find_t_step(foot_plan, r_f, t_step);

    find_r_vrp_d(r_f, delta_z, r_vrp_d);
    find_dcm_d_eos(r_vrp_d, omega, t_step, dcm_d_eos);
    find_dcm_d(r_vrp_d, omega, t_step, dcm_d_eos, dcm_d, d_dcm_d);
    find_x_com_trajectory(robot_state,
                          dcm_d,
                          d_dcm_d,
                          omega,
                          x_com_traj_out,
                          d_x_com_traj_out,
                          dd_x_com_traj_out);
}

void DCMGenerator::find_r_f(const FootPlanData& foot_plan,
                            const RobotData& robot_state,
                            std::vector<Eigen::Vector3d>& r_f)  {
    // определим именно те следовые точки, которые не равны нулю
    std::vector<std::array<Eigen::Vector3d, 2>> support_segments;
    support_segments.reserve(PREVIEW_STRIDES_HORIZON);

    for (int stride = 0; stride < PREVIEW_STRIDES_HORIZON; ++stride) {
        std::array<Eigen::Vector3d, 2> support_segment;
        int support_count = 0;

        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            if (foot_plan.duration[stride][leg] <= 0.0) {
                continue;
            }

            if (support_count >= 2) {
                throw std::runtime_error("find_r_f expects exactly two support legs per stride but they are greater");
            }
            

            support_segment[support_count] = Eigen::Vector3d(
                foot_plan.foot_sequence[stride][leg][X],
                foot_plan.foot_sequence[stride][leg][Y],
                foot_plan.foot_sequence[stride][leg][Z]);
            ++support_count;
        }

        // cout << support_count << endl;

        if (support_count != 2) {
            throw std::runtime_error("find_r_f expects exactly two support legs per stride");
        }

        support_segments.push_back(support_segment);
    }

    // 
    r_f.clear();
    r_f.reserve(PREVIEW_STRIDES_HORIZON);

    first_support_p1 = support_segments.front()[0];
    first_support_p2 = support_segments.front()[1];
    r_f.push_back(closest_point_on_segment_to_body(robot_state.pos, first_support_p1, first_support_p2));

    for (int i = 0; i < PREVIEW_STRIDES_HORIZON - 1; ++i) {
        const Eigen::Vector3d& p1 = support_segments[i][0];
        const Eigen::Vector3d& p2 = support_segments[i][1];
        const Eigen::Vector3d& p3 = support_segments[i + 1][0];
        const Eigen::Vector3d& p4 = support_segments[i + 1][1];

        const std::optional<Eigen::Vector2d> intersection =
            segment_intersection(p1.head<2>(), p2.head<2>(), p3.head<2>(), p4.head<2>());

        if (!intersection.has_value()) {
            throw std::runtime_error("support segments do not intersect, cannot define r_f");
        }

        const double z_intersection = 0.25 * (p1(Z) + p2(Z) + p3(Z) + p4(Z));
        r_f.emplace_back((*intersection)(X), (*intersection)(Y), z_intersection);
    }
}

void DCMGenerator::find_r_vrp_d(const std::vector<Eigen::Vector3d>& r_f,
                                const double& delta_z,
                                std::vector<Eigen::Vector3d>& r_vrp_d) {
    r_vrp_d.clear();
    r_vrp_d.reserve(r_f.size());

    for (const auto& rf : r_f) {
        r_vrp_d.emplace_back(rf(X), rf(Y), rf(Z) + delta_z);
    }
}

void DCMGenerator::find_t_step(const FootPlanData& foot_plan,
                               const std::vector<Eigen::Vector3d>& r_f,
                               std::vector<double>& t_step) {
    t_step.clear();

    if (r_f.size() < 2) {
        throw std::runtime_error("find_t_step expects at least two r_f points");
    }

    t_step.reserve(r_f.size() - 1);
    for (std::size_t stride = 0; stride + 1 < r_f.size(); ++stride) {
        double stride_duration = 0.0;
        bool duration_found = false;

        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            const double duration = foot_plan.duration[stride][leg];
            if (duration <= 0.0) {
                continue;
            }

            if (!std::isfinite(duration)) {
                throw std::runtime_error("find_t_step expects finite step durations");
            }

            if (!duration_found) {
                stride_duration = duration;
                duration_found = true;
                continue;
            }

            if (std::abs(duration - stride_duration) > 1e-6) {
                throw std::runtime_error("find_t_step expects equal durations for support legs");
            }
        }

        if (!duration_found) {
            throw std::runtime_error("find_t_step cannot define step duration from foot plan");
        }

        t_step.push_back(stride_duration);
    }
}

void DCMGenerator::find_dcm_d_eos(const std::vector<Eigen::Vector3d>& r_vrp_d,
                                    const double& omega,
                                    const std::vector<double>& t_step,
                                    std::vector<Eigen::Vector3d>& dcm_d_eos) {
    dcm_d_eos.clear();

    const std::size_t vrp_count = r_vrp_d.size();
    if (vrp_count < 2) {
        throw std::runtime_error("find_dcm_d_eos expects at least two VRP points");
    }

    if (t_step.size() < vrp_count - 1) {
        throw std::runtime_error("find_dcm_d_eos expects at least r_vrp_d.size() - 1 step durations");
    }

    dcm_d_eos.resize(vrp_count, Eigen::Vector3d::Zero());

    dcm_d_eos[vrp_count - 1] = r_vrp_d[vrp_count - 1];
    dcm_d_eos[vrp_count - 2] = r_vrp_d[vrp_count - 1];

    for (std::size_t i = vrp_count - 2; i > 0; --i) {
        const double step_decay = std::exp(-omega * t_step[i]);
        dcm_d_eos[i - 1] = r_vrp_d[i] + step_decay * (dcm_d_eos[i] - r_vrp_d[i]);
    }
}

void DCMGenerator::find_dcm_d(const std::vector<Eigen::Vector3d>& r_vrp_d,
                    const double& omega,
                    const std::vector<double>& t_step,
                    const std::vector<Eigen::Vector3d>& dcm_d_eos,
                    std::vector<Eigen::Vector3d>& dcm_d,
                    std::vector<Eigen::Vector3d>& d_dcm_d) {
    dcm_d.clear();
    d_dcm_d.clear();

    const std::size_t vrp_count = r_vrp_d.size();
    if (vrp_count < 2) {
        throw std::runtime_error("find_dcm_d expects at least two VRP points");
    }

    if (t_step.size() < vrp_count - 1) {
        throw std::runtime_error("find_dcm_d expects at least r_vrp_d.size() - 1 step durations");
    }

    if (dcm_d_eos.size() < vrp_count - 1) {
        throw std::runtime_error("find_dcm_d expects at least r_vrp_d.size() - 1 DCM EOS points");
    }

    if (dt <= 0.0) {
        throw std::runtime_error("find_dcm_d expects positive dt");
    }

    for (std::size_t i = 0; i < vrp_count - 1; ++i) {
        for (double t = -t_step[i]; t <= 0.0; t += dt) {
            const double step_growth = std::exp(omega * t);
            const Eigen::Vector3d dcm_point =
                r_vrp_d[i] + step_growth * (dcm_d_eos[i] - r_vrp_d[i]);
            dcm_d.push_back(dcm_point);
            d_dcm_d.push_back(omega * (dcm_point - r_vrp_d[i]));
        }
    }
}

void DCMGenerator::find_x_com_trajectory(const RobotData& robot_state,
                                        const std::vector<Eigen::Vector3d>& dcm_d,
                                        const std::vector<Eigen::Vector3d>& d_dcm_d,
                                        const double omega,
                                        std::vector<Eigen::Vector3d>& x_com_traj,
                                        std::vector<Eigen::Vector3d>& d_x_com_traj,
                                        std::vector<Eigen::Vector3d>& dd_x_com_traj) {
    x_com_traj.clear();
    d_x_com_traj.clear();
    dd_x_com_traj.clear();

    if (dcm_d.size() < 2) {
        throw std::runtime_error("find_x_com_trajectory expects at least two DCM points");
    }

    if (d_dcm_d.size() < dcm_d.size()) {
        throw std::runtime_error("find_x_com_trajectory expects one DCM velocity point per DCM point");
    }

    if (dt <= 0.0) {
        throw std::runtime_error("find_x_com_trajectory expects positive dt");
    }

    if (!std::isfinite(omega) || omega <= 0.0) {
        throw std::runtime_error("find_x_com_trajectory expects positive finite omega");
    }

    x_com_traj.reserve(dcm_d.size() + 1);
    d_x_com_traj.reserve(dcm_d.size());
    dd_x_com_traj.reserve(dcm_d.size());

    x_com_traj.push_back(robot_state.pos);

    for (std::size_t i = 0; i < dcm_d.size(); ++i) {
        const Eigen::Vector3d d_x_com = omega * (dcm_d[i] - x_com_traj[i]);
        const Eigen::Vector3d dd_x_com = omega * (d_dcm_d[i] - d_x_com);

        d_x_com_traj.push_back(d_x_com);
        dd_x_com_traj.push_back(dd_x_com);
        x_com_traj.push_back(x_com_traj[i] + dt * d_x_com);
        // x_com_traj.push_back(x_com_traj[i] + (1 - exp(-omega * dt)) * (dcm_d[i] - x_com_traj[i]));
    }

}

std::optional<Eigen::Vector2d> DCMGenerator::segment_intersection(
    const Eigen::Vector2d& p1,
    const Eigen::Vector2d& p2,
    const Eigen::Vector2d& p3,
    const Eigen::Vector2d& p4) const {

    const double d = (p1(X) - p2(X)) * (p3(Y) - p4(Y)) - (p1(Y) - p2(Y)) * (p3(X) - p4(X));
    if (std::abs(d) < 1e-12) {
        return std::nullopt;
    }

    const double det12 = p1(X) * p2(Y) - p1(Y) * p2(X);
    const double det34 = p3(X) * p4(Y) - p3(Y) * p4(X);
    const double x = (det12 * (p3(X) - p4(X)) - (p1(X) - p2(X)) * det34) / d;
    const double y = (det12 * (p3(Y) - p4(Y)) - (p1(Y) - p2(Y)) * det34) / d;

    return Eigen::Vector2d(x, y);
}

Eigen::Vector3d DCMGenerator::closest_point_on_segment_to_body(
    const Eigen::Vector3d& body_pos,
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2) const {

    const Eigen::Vector2d segment_xy = p2.head<2>() - p1.head<2>();
    const double segment_length_sq = segment_xy.dot(segment_xy);

    if (segment_length_sq < 1e-12) {
        return p1;
    }

    double t = (body_pos.head<2>() - p1.head<2>()).dot(segment_xy) / segment_length_sq;
    t = std::clamp(t, 0.0, 1.0);

    return p1 + t * (p2 - p1);
}
