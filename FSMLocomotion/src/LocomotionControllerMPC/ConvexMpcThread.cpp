#include "ConvexMpcThread.hpp"

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#include <cstring>
#include <unistd.h>
#endif

namespace {
#ifdef __linux__
void configure_realtime_thread(const char* thread_name,
                               pthread_t handle,
                               int priority,
                               int cpu_index)
{
    sched_param sp{};
    sp.sched_priority = priority;
    const int sched_rc = pthread_setschedparam(handle, SCHED_FIFO, &sp);
    if (sched_rc != 0) {
        std::cout << '[' << thread_name << "] pthread_setschedparam failed: "
                  << std::strerror(sched_rc) << '\n';
    } else {
        std::cout << '[' << thread_name << "] SCHED_FIFO priority=" << priority << '\n';
    }

    const long cpu_count = sysconf(_SC_NPROCESSORS_ONLN);
    if (cpu_count <= 0) {
        std::cout << '[' << thread_name << "] sysconf(_SC_NPROCESSORS_ONLN) failed\n";
        return;
    }

    if (cpu_index < 0 || cpu_index >= cpu_count) {
        std::cout << '[' << thread_name << "] requested CPU " << cpu_index
                  << " is out of range 0-" << (cpu_count - 1) << '\n';
        return;
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_index, &cpuset);
    const int affinity_rc = pthread_setaffinity_np(handle, sizeof(cpuset), &cpuset);
    if (affinity_rc != 0) {
        std::cout << '[' << thread_name << "] pthread_setaffinity_np failed: "
                  << std::strerror(affinity_rc) << '\n';
    } else {
        std::cout << '[' << thread_name << "] affinity cpu=" << cpu_index << '\n';
    }
}
#endif
}

ConvexMPCThread::ConvexMPCThread()
{
    ref_grf.resize(12);
    x0.resize(13);
    foot_positions.resize(3, 4);
    x_ref.resize(13);
    phase_signal.resize(4);

    ref_grf.setZero();
    x0.setZero();
    foot_positions.setZero();
    x_ref.setZero();

    en = false;
    standing = true;
    phi0 = 0.0;
    module_dt = 0.0;
    dt = std::chrono::duration<double>::zero();
    tick_period = std::chrono::steady_clock::duration::zero();
    running = false;

    active_legs = {true, true, true, true};
}

ConvexMPCThread::~ConvexMPCThread() { stop_thread(); }

void ConvexMPCThread::stop_thread()
{
    running = false;
    if (worker_thread.joinable()) {
        worker_thread.join();
    }
}

auto ConvexMPCThread::now()
{
    return std::chrono::steady_clock::now();
}

void ConvexMPCThread::callback()
{
    RobotData robot_state_snapshot;
    LegData leg_state_snapshot;
    VectorXd x_ref_snapshot(13);
    vector<int> phase_signal_snapshot(4);
    vector<bool> active_legs_snapshot(NUM_LEGS, true);
    Gait gait_snapshot;
    bool en_snapshot = false;
    bool standing_snapshot = true;
    double phi0_snapshot = 0.0;

    auto next_tick = now() + tick_period;

    std::uint64_t solver_generation = 0;
    while (running)
    {
        std::uint64_t job_generation;
        {
            lock_guard<mutex> lock(data_mutex);
            job_generation = generation;
            robot_state_snapshot = robot_state;
            leg_state_snapshot = leg_state;
            x_ref_snapshot = x_ref;
            en_snapshot = en;
            standing_snapshot = standing;
            phase_signal_snapshot = phase_signal;
            phi0_snapshot = phi0;
            active_legs_snapshot = active_legs;
            gait_snapshot = gait;
        }

        if (solver_generation != job_generation) {
            mpc.reset_solution();
            gait_scheduler.reset_mpc_table();
            solver_generation = job_generation;
        }
        if (en_snapshot) {
            gait_scheduler.set_gait_params(gait_snapshot.stance, gait_snapshot.swing,
                                           gait_snapshot.offsets, gait_snapshot.initial);

            x0 << robot_state_snapshot.orientation(X),
                  robot_state_snapshot.orientation(Y),
                  robot_state_snapshot.orientation(Z),
                  robot_state_snapshot.pos(X),
                  robot_state_snapshot.pos(Y),
                  robot_state_snapshot.pos(Z),
                  robot_state_snapshot.ang_vel(X),
                  robot_state_snapshot.ang_vel(Y),
                  robot_state_snapshot.ang_vel(Z),
                  robot_state_snapshot.lin_vel(X),
                  robot_state_snapshot.lin_vel(Y),
                  robot_state_snapshot.lin_vel(Z),
                  -robot.g;

            foot_positions.col(0) = leg_state_snapshot.r1_pos - robot_state_snapshot.pos;
            foot_positions.col(1) = leg_state_snapshot.l1_pos - robot_state_snapshot.pos;
            foot_positions.col(2) = leg_state_snapshot.r2_pos - robot_state_snapshot.pos;
            foot_positions.col(3) = leg_state_snapshot.l2_pos - robot_state_snapshot.pos;

            gait_scheduler.getMpcTable(phi0_snapshot, standing_snapshot,
                                       phase_signal_snapshot, active_legs_snapshot,
                                       gait_table);

            const VectorXd ref_grf_local =
                mpc.get_contact_forces(x0, x_ref_snapshot, foot_positions, gait_table);

            lock_guard<mutex> lock(data_mutex);
            if (job_generation == generation && ref_grf_local.size() == 12 && ref_grf_local.allFinite()) {
                ref_grf = ref_grf_local;
                if (mpc.converged()) converged_grf = ref_grf_local;
            }
        }

        const auto current_time = now();
        if (current_time < next_tick) {
            std::this_thread::sleep_until(next_tick);
            next_tick += tick_period;
            continue;
        }

        const std::chrono::duration<double, std::milli> lateness{current_time - next_tick};
        if (lateness.count() > 0.05) {
            cout << "[LocomotionController->ConvexMPCThread]: Overrun: "
                 << lateness.count() << " ms" << endl;
        }

        do {
            next_tick += tick_period;
        } while (next_tick <= current_time);
    }
}

void ConvexMPCThread::start_thread()
{
    if (running) return;
    gait_scheduler.reset();
    gait_scheduler.reset_mpc_table();

    running = true;
    worker_thread = std::thread(&ConvexMPCThread::callback, this);

#ifdef __linux__
    configure_realtime_thread("LocomotionController->ConvexMPCThread",
                              worker_thread.native_handle(),
                              70,
                              2);
#endif
}

void ConvexMPCThread::set_physical_params(RobotPhysicalParams& robot)
{
    mpc.set_physical_params(robot);
    this->robot = robot;
}

void ConvexMPCThread::set_mpc_params(double timestep, int horizon, double friction_coeff,
                        double f_min, double f_max, VectorXd &Q, VectorXd &R)
{
    mpc.set_mpc_params(timestep, horizon, friction_coeff, f_min, f_max, Q, R);
    gait_table.resize(NUM_LEGS * horizon);
    this->module_dt = timestep;
    dt = std::chrono::duration<double>(module_dt);
    tick_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(dt);

    gait_scheduler.setMpcParams(timestep, horizon);
}

void ConvexMPCThread::set_gait_params(double t_st,
                        double t_sw,
                        const std::vector<double>& phase_offsets,
                        const std::vector<int>& phase_init)
{
    lock_guard<mutex> lock(data_mutex);
    pending_gait.stance = t_st;
    pending_gait.swing = t_sw;
    pending_gait.offsets = phase_offsets;
    pending_gait.initial = phase_init;
}

void ConvexMPCThread::set_observation_data(const RobotData& robot_state, const LegData& leg_state, const VectorXd& x_ref,
                        const MatrixXd& R_body, const bool& en, const bool& standing, const std::vector<int>& phase_signal,
                        const double& phi0, const vector<bool>& active_legs)
{
    (void)R_body;

    lock_guard<mutex> lock(data_mutex);
    // Весь прогноз, включая длительности фаз, относится к одному наблюдению.
    gait = pending_gait;
    this->robot_state = robot_state;
    this->leg_state = leg_state;
    this->x_ref = x_ref;
    this->en = en;
    this->standing = standing;
    this->phase_signal = phase_signal;
    this->phi0 = phi0;
    this->active_legs = active_legs;
}

VectorXd ConvexMPCThread::get_ref_grf()
{
    lock_guard<mutex> lock(data_mutex);
    return ref_grf;
}

void ConvexMPCThread::reset()
{
    lock_guard lock(data_mutex);
    ++generation;
    en = false;
    converged_grf.reset();
    ref_grf.setZero();
}

std::optional<VectorXd> ConvexMPCThread::solution()
{
    lock_guard lock(data_mutex);
    return converged_grf;
}
