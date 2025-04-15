#ifndef GMBASEDFORCEOBSERVER_HPP
#define GMBASEDFORCEOBSERVER_HPP

#include <Eigen/Dense>
#include "LowPassFilter.hpp"

class GMBasedForceObserver {
public:
    GMBasedForceObserver();

    void set_params(double lamb, double dt, const Eigen::VectorXd& p);
    void set_p(const Eigen::VectorXd& p);

    Eigen::VectorXd step(const Eigen::VectorXd& tau, const Eigen::VectorXd& dq,
                         const Eigen::MatrixXd& M, const Eigen::VectorXd& C,
                         const Eigen::VectorXd& G, const Eigen::MatrixXd& J);

private:
    double lamb;
    double gamma;
    double beta;
    Eigen::VectorXd p;
    Eigen::VectorXd p_k, tau_dist, f_hat;
    Eigen::VectorXd w, w_;
    Eigen::MatrixXd invJ_T;
    double w0, w1, w2;
    LowPassFilter lpf0, lpf1, lpf2; // Предполагается, что класс LowPassFilter уже реализован
};

#endif // GMBASEDFORCEOBSERVER_HPP