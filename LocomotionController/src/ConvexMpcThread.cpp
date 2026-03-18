#include "ConvexMpcThread.hpp"

ConvexMPCThread::ConvexMPCThread()
{
    ref_grf.resize(12);
    x0.resize(13);
    foot_positions.resize(3, 4);

    ref_grf.setZero();
    x0.setZero();
    foot_positions.setZero();

    rpy_rate.resize(3);
    com_vel_body_frame.resize(3);
    x_ref.resize(13);
    des_state.resize(13);
    phase_signal.resize(4);

    en = false;
    standing = true;
    
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
            rpy_rate = robot_state.ang_vel; 

            x0 <<   robot_state.orientation(X),
                    robot_state.orientation(Y),
                    robot_state.orientation(Z),
                    robot_state.pos(X),
                    robot_state.pos(Y),
                    robot_state.pos(Z),
                    rpy_rate(X), 
                    rpy_rate(Y), 
                    rpy_rate(Z), 
                    robot_state.lin_vel(X), 
                    robot_state.lin_vel(Y), 
                    robot_state.lin_vel(Z), 
                    -robot.g;

            assert(leg_state.r1_pos.rows() == 3);

            foot_positions.col(0) = (leg_state.r1_pos - robot_state.pos); 
            foot_positions.col(1) = (leg_state.l1_pos - robot_state.pos);
            foot_positions.col(2) = (leg_state.r2_pos - robot_state.pos);
            foot_positions.col(3) = (leg_state.l2_pos - robot_state.pos);

            // predict future contact states
            gait_scheduler.reset_mpc_table();
            gait_table = gait_scheduler.getMpcTable(phi0, standing, phase_signal, active_legs);

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
    gait_scheduler.reset();
    gait_scheduler.reset_mpc_table();
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