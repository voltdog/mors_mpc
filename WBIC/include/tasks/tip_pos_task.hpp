#pragma once
#include "wbc_task.hpp"

class TipPosTask : public WBCTask
{
private:

    int leg_id;

    // Eigen::Matrix3d kp;
    // Eigen::Matrix3d kd;

public:

    TipPosTask(int leg);

    void update(
        Robot& model,
        const ConfigVector& ref_x_wcs,
        const GenCoordVector& ref_xdot_wcs,
        const GenCoordVector& ref_xddot_wcs) override;
};
