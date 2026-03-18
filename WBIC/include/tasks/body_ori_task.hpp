#pragma once
#include "wbc_task.hpp"

class BodyOriTask : public WBCTask
{
private:

    // Eigen::Matrix3d kp;
    // Eigen::Matrix3d kd;

public:

    BodyOriTask();

    void update(
        Robot& model,
        const ConfigVector& ref_x_wcs,
        const GenCoordVector& ref_xdot_wcs,
        const GenCoordVector& ref_xddot_wcs) override;
};
