#pragma once
#include "wbc_task.hpp"

class BodyPosTask : public WBCTask
{
private:

    // Eigen::Matrix3d kp;
    // Eigen::Matrix3d kd;

public:

    BodyPosTask();

    void update(
        Robot& model,
        const ConfigVector& ref_x_wcs,
        const GenCoordVector& ref_xdot_wcs,
        const GenCoordVector& ref_xddot_wcs) override;
};
