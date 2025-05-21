#include "GaitTransition.hpp"

GaitTransition::GaitTransition()
{
    
}

void GaitTransition::set_transition_duration(double delta_T_gt)
{
    this->delta_T_gt = delta_T_gt;

    pre_t_st = t_st;
    pre_t_sw = t_sw;
    pre_phase_offsets = phase_offsets;
    transisting = false;
}

void GaitTransition::set_gait_params(double t_st,
                                    double t_sw,
                                    std::vector<double>& phase_offsets)
{
    if (t_st != pre_t_st || t_sw != pre_t_sw || phase_offsets != pre_phase_offsets)
    {
        transisting = true;
        t_st_old = this->t_st;
        t_sw_old = this->t_sw;
        phase_offsets_old = this->phase_offsets;
    }

    this->t_st = t_st;
    this->t_sw = t_sw;
    this->phase_offsets = phase_offsets;
}

double GaitTransition::saturation(double x, double y)
{
    if (x <= y)
        return x;
    else
        return y;
}

void GaitTransition::make_transition(double t, 
                                    double& t_st_out,
                                    double& t_sw_out,
                                    std::vector<double>& phase_offsets_out)
{
    if (t_st != pre_t_st || t_sw != pre_t_sw || phase_offsets != pre_phase_offsets)
    {
        t_gt = t;
    }

    if (transisting == true)
    {
        t_saturation = (t - t_gt) / delta_T_gt;
        t_sw_out = t_sw_old + (t_sw - t_sw_old) * saturation(t_saturation, 1.0);
        t_st_out = t_st_old + (t_st - t_st_old) * saturation(t_saturation, 1.0);

        for (int i = 0; i < 4; i++)
        {
            phase_offsets_out[i] = phase_offsets_old[i] + (phase_offsets[i] - phase_offsets_old[i]) * saturation(t_saturation, 1.0);
        }

        if (t_saturation >= 1.0)
        {
            transisting = false;
            // cout << "hoi" << endl;
        }
        // cout << "(" << t_saturation << " = " << t << " - " << t_gt << ") / " << delta_T_gt << endl;
        // cout << t_sw_out << " " << t_st_out << endl;
        // cout << "---" << endl;
    }
    else
    {
        t_sw_out = t_sw;
        t_st_out = t_st;
        phase_offsets_out = phase_offsets;
    }


    pre_t_st = t_st;
    pre_t_sw = t_sw;
    pre_phase_offsets = phase_offsets;
}