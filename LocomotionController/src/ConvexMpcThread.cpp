#include "ConvexMpcThread.hpp"

// #define X 0
// #define Y 1
// #define Z 2

ConvexMPCThread::ConvexMPCThread()
{
    ref_grf.resize(12);
    x0.resize(13);
    foot_positions.resize(3, 4);

    ref_grf.setZero();
    x0.setZero();
    foot_positions.setZero();

    robot_state.ang_vel.resize(3);
    robot_state.lin_vel.resize(3);
    robot_state.orientation.resize(3);
    robot_state.pos.resize(3);
    leg_state.r1_pos.resize(3);
    leg_state.l1_pos.resize(3);
    leg_state.r2_pos.resize(3);
    leg_state.l2_pos.resize(3);
    leg_state.r1_vel.resize(3);
    leg_state.l1_vel.resize(3);
    leg_state.r2_vel.resize(3);
    leg_state.l2_vel.resize(3);
    leg_state.r1_acc.resize(3);
    leg_state.l1_acc.resize(3);
    leg_state.r2_acc.resize(3);
    leg_state.l2_acc.resize(3);
    leg_state.r1_grf.resize(3);
    leg_state.l1_grf.resize(3);
    leg_state.r2_grf.resize(3);
    leg_state.l2_grf.resize(3);
    leg_state.r1_kp.resize(3);
    leg_state.l1_kp.resize(3);
    leg_state.r2_kp.resize(3);
    leg_state.l2_kp.resize(3);
    leg_state.r1_kd.resize(3);
    leg_state.l1_kd.resize(3);
    leg_state.r2_kd.resize(3);
    leg_state.l2_kd.resize(3);
    rpy_rate.resize(3);
    com_vel_body_frame.resize(3);
    x_ref.resize(13);
    des_state.resize(13);
    R_body.resize(3,3);
    R_z.resize(3,3);
    en = false;
    standing = true;
    phase_signal.resize(4);
    phi0 = 0.0;

    active_legs = {true, true, true, true};
}

ConvexMPCThread::~ConvexMPCThread()
{

}

// current time
auto ConvexMPCThread::now() 
{
  return std::chrono::steady_clock::now(); 
}

void ConvexMPCThread::callback()
{
    // cout << "1" << endl;
    while(true)
    {
        // Calculating current time
        const auto start{ now() };

        if (en == true) {
            // form x0 vector
            // попробовать rpy_rate, как main у swing контроллера
            cos_yaw = cos(robot_state.orientation(Z));
            sin_yaw = sin(robot_state.orientation(Z));
            R_z   << cos_yaw, -sin_yaw, 0, 
                    sin_yaw,  cos_yaw, 0,  
                    0,              0, 1;
            rpy_rate = R_z * robot_state.ang_vel; //
            // com_vel_body_frame = R_body.transpose() * robot_state.lin_vel;
            x0 <<   robot_state.orientation(X),
                    robot_state.orientation(Y),
                    robot_state.orientation(Z),
                    // ((x_ref(8) == 0.0) ? (robot_state.orientation(Z) - x_ref(2)) : (robot_state.orientation(Z) - pre_yaw)),
                    robot_state.pos(X),
                    robot_state.pos(Y),
                    robot_state.pos(Z),
                    rpy_rate(X), // robot_state.ang_vel(X), // 
                    rpy_rate(Y), //robot_state.ang_vel(Y),
                    rpy_rate(Z), //robot_state.ang_vel(Z),
                    robot_state.lin_vel(X), //com_vel_body_frame(X), //
                    robot_state.lin_vel(Y), //com_vel_body_frame(Y), //
                    robot_state.lin_vel(Z), //com_vel_body_frame(Z), //
                    -robot.g;

            // des_state << x_ref(0),
            //             x_ref(1),
            //             // x_ref(2),
            //             ((x_ref(8) == 0.0) ? (0.0) : (robot_state.orientation(Z) - pre_yaw)) + module_dt * x_ref(8),
            //             x_ref(3),
            //             x_ref(4),
            //             x_ref(5),
            //             x_ref(6),
            //             x_ref(7),
            //             x_ref(8),
            //             x_ref(9),
            //             x_ref(10),
            //             x_ref(11),
            //             x_ref(12),
            // form foot_positions vector
            // std::cout << "R_body: " << R_body.rows() << "x" << R_body.cols() << std::endl;
            // std::cout << "r1_pos: " << leg_state.r1_pos.rows() << "x" << leg_state.r1_pos.cols() << std::endl;
            assert(leg_state.r1_pos.rows() == 3);

            // cos_yaw = cos(robot_state.orientation(Z));
            // sin_yaw = sin(robot_state.orientation(Z));
            // R_z   << cos_yaw, -sin_yaw, 0, 
            //         sin_yaw,  cos_yaw, 0,  
            //         0,              0, 1;

            foot_positions.col(0) = R_z * leg_state.r1_pos; //R_body
            foot_positions.col(1) = R_z * leg_state.l1_pos; // R_z
            foot_positions.col(2) = R_z * leg_state.r2_pos;
            foot_positions.col(3) = R_z * leg_state.l2_pos;

            // predict future contact states
            // cout << "1" << endl;
            gait_scheduler.reset_mpc_table();
            // cout << "1" << endl;
            gait_table = gait_scheduler.getMpcTable(phi0, standing, phase_signal, active_legs);
            // cout << "1" << endl;

            // if (t > 15.54 && t < 15.8)
            // {
            //     cout << "t: " << t << endl;
            //     cout << "standing: " << standing << endl;
            //     for (int i=0; i < 4; i++)
            //     {
            //         cout << phase_signal[i] << " ";
            //     }
            //     cout << endl;
            //     cout << "-" << endl;
            //     for (int i=0; i < static_cast<int>(gait_table.size()); i++)
            //     {
            //         cout << gait_table[i] << " ";
            //         if ((i+1) % 4 == 0)
            //             cout << "| ";
            //     }
            //     cout << endl;
            //     cout << "===" << endl;
            // }

            // solve mpc problem
            ref_grf = mpc.get_contact_forces(x0, x_ref, foot_positions, gait_table);

            pre_yaw = robot_state.orientation(Z);
        }
        // Wait until spinning time
        while(true)
        {
            std::chrono::duration<double, std::milli> elapsed{now() - start};
            if (elapsed >= dt)
            {
                if (elapsed.count() > 1000*module_dt+0.05)
                    cout << "[LocomotionController->ConvexMPCThread]: Waited for : " << elapsed.count() << " ms" << endl;
                break;
            }
        }
    }


}

void ConvexMPCThread::start_thread()
{
    // cout << "2" << endl;
    gait_scheduler.reset();
    // cout << "2" << endl;
    gait_scheduler.reset_mpc_table();
    // cout << "2" << endl;
    std::thread t(&ConvexMPCThread::callback, this); // создаем поток
    t.detach(); // ждем завершения потока
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
    gait_table.resize(4 * horizon);
    this->module_dt = timestep;
    dt = std::chrono::duration<double>(module_dt);

    gait_scheduler.setMpcParams(timestep, horizon);
    
}

void ConvexMPCThread::set_gait_params(double t_st,
                        double t_sw,
                        const std::vector<double>& phase_offsets,
                        const std::vector<int>& phase_init)
{
    gait_scheduler.set_timestep(0.002);
    gait_scheduler.set_gait_params(t_st, t_sw, phase_offsets, phase_init);
    // gait_scheduler.reset();
    // gait_scheduler.reset_mpc_table();
}

void ConvexMPCThread::set_observation_data(RobotData& robot_state, LegData& leg_state, VectorXd& x_ref, 
                        MatrixXd& R_body, bool& en, bool& standing, std::vector<int>& phase_signal, 
                        double& phi0, vector<bool> active_legs)
{
    this->robot_state = robot_state;
    this->leg_state = leg_state;
    this->x_ref = x_ref;
    this->R_body = R_body;
    this->en = en;
    this->standing = standing;
    this->phase_signal = phase_signal;
    this->phi0 = phi0;
    this->active_legs = active_legs;
}

VectorXd ConvexMPCThread::get_ref_grf()
{
    return ref_grf;
}