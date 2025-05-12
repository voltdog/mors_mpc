#include "SimpleGaitScheduler.hpp"
#include <Eigen/Dense>
#include <cmath>

SimpleGaitScheduler::SimpleGaitScheduler()
{

}

void SimpleGaitScheduler::set_gait_params(double T_st,
    double T_sw,
    const std::vector<double>& phase_offsets,
    const std::vector<int>& phase_init)
{
    this->T_st_ = T_st;
    this->T_sw_ = T_sw;
    this->phase_offsets_ = phase_offsets;
    this->phase_init_ = phase_init;

    num_legs_ = static_cast<int>(phase_offsets.size());
    full_cycle_period_ = T_st_ + T_sw_;
    duty_factor_ = T_st_ / full_cycle_period_;

    initial_state_ratio_in_cycle_.resize(num_legs_, 0.0);
    next_leg_state_ = {STANCE, STANCE, STANCE, STANCE};

    for (int leg = 0; leg < num_legs_; ++leg) {
        if (phase_init_[leg] == SWING) {
            initial_state_ratio_in_cycle_[leg] = 1.0 - duty_factor_;
            next_leg_state_[leg] = STANCE;
        } else {
            initial_state_ratio_in_cycle_[leg] = duty_factor_;
            next_leg_state_[leg] = SWING;
        }
    }

    // standing_phase = {-1,-1,-1,-1};

    // reset();
}
void SimpleGaitScheduler::reset()
{
    normalized_phase_.assign(num_legs_, 0.0);
    leg_state_ = phase_init_;
    mpc_leg_state_ = phase_init_;
    desired_leg_state_ = phase_init_;
}

void SimpleGaitScheduler::step(double t, bool standing, std::vector<int>& leg_state, std::vector<double>& leg_phase)
{
    for (int leg = 0; leg < num_legs_; ++leg) 
    {
        double augmented_time = t + phase_offsets_[leg] * full_cycle_period_;
        double phase_in_full_cycle = std::fmod(augmented_time, full_cycle_period_) / full_cycle_period_;
        double ratio = initial_state_ratio_in_cycle_[leg];

        
        if (standing == true)
        {
            if (phase_in_full_cycle < ratio) {
                desired_leg_state_[leg] = phase_init_[leg];
                normalized_phase_[leg] = phase_in_full_cycle / ratio;
            }
            else {
                // desired_leg_state_[leg] = next_leg_state_[leg];
                normalized_phase_[leg] = (phase_in_full_cycle - ratio) / (1.0 - ratio);
            }
        }
        else
        {
            if (phase_in_full_cycle < ratio) {
                desired_leg_state_[leg] = phase_init_[leg];
                normalized_phase_[leg] = phase_in_full_cycle / ratio;
            } else {
                desired_leg_state_[leg] = next_leg_state_[leg];
                normalized_phase_[leg] = (phase_in_full_cycle - ratio) / (1.0 - ratio);
            }
        }

        leg_state[leg] = desired_leg_state_[leg];
        leg_phase[leg] = normalized_phase_[leg];
        
    }
    // cout << leg_state[0] << " " << leg_state[1] << " " << leg_state[2] << " " << leg_state[3] << endl;

}

void SimpleGaitScheduler::setMpcParams(double dt_mpc, int n_horizon)
{
    this->dt_mpc_ = dt_mpc;
    this->n_horizon_ = n_horizon;
}

vector<int> SimpleGaitScheduler::getMpcTable(double t0, bool standing, const std::vector<int>& current_leg_state, vector<double>& current_leg_phase)
{
    vector<int> gait_table(num_legs_ * n_horizon_);
    double t = t0;

    // if (current_leg_phase == standing_phase)
    // {
    //     std::fill(gait_table.begin(), gait_table.end(), STANCE);
    // }
    // else
    // {
        for (int i = 0; i < n_horizon_; ++i) {
            for (int leg = 0; leg < num_legs_; ++leg) {
                double augmented_time = t + phase_offsets_[leg] * full_cycle_period_;
                double phase_in_full_cycle = std::fmod(augmented_time, full_cycle_period_) / full_cycle_period_;
                double ratio = initial_state_ratio_in_cycle_[leg];
                
                if (standing == true)
                {
                    if (phase_in_full_cycle < ratio) 
                        mpc_leg_state_[leg] = phase_init_[leg];
                }
                else
                {
                    if (phase_in_full_cycle < ratio) {
                        mpc_leg_state_[leg] = phase_init_[leg];
                    } else {
                        mpc_leg_state_[leg] = next_leg_state_[leg];
                    }
                }
    
                if (current_leg_state[leg] == EARLY_CONTACT) {
                    mpc_leg_state_[leg] = STANCE;
                }
                
                if (i == 0)
                {
                    if (current_leg_state[leg] == LATE_CONTACT) {
                        mpc_leg_state_[leg] = SWING;
                    }
                }
    
                gait_table[i * num_legs_ + leg] = mpc_leg_state_[leg];
            }
            t += dt_mpc_;
        }
    // }
    

    return gait_table;
}