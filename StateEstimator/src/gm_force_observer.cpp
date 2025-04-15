#include "gm_force_observer.hpp"
#include <Eigen/LU> // Для вычисления обратной матрицы

GMBasedForceObserver::GMBasedForceObserver() {
    // Конструктор по умолчанию
    w.resize(3);
    w_.resize(3);
}

void GMBasedForceObserver::set_params(double lamb, double dt, const Eigen::VectorXd& p) {
    this->lamb = lamb;
    // this->gamma = std::exp(-lamb * dt);
    // this->beta = (1 - this->gamma) / (this->gamma * dt);
    this->p = p;

    this->lpf0.reconfigureFilter(dt, lamb);
    this->lpf1.reconfigureFilter(dt, lamb);
    this->lpf2.reconfigureFilter(dt, lamb);
}

void GMBasedForceObserver::set_p(const Eigen::VectorXd& p)
{
    this->p = p;
}

Eigen::VectorXd GMBasedForceObserver::step(const Eigen::VectorXd& tau, const Eigen::VectorXd& dq,
                                           const Eigen::MatrixXd& M, const Eigen::VectorXd& C,
                                           const Eigen::VectorXd& G, const Eigen::MatrixXd& J) {
    // Вычисление p_k
    p_k = M * dq;

    // Вычисление w_
    w_ = this->lamb * this->p + tau + C - G;

    // Обновление фильтра
    w0 = this->lpf0.update(w_(0));
    w1 = this->lpf1.update(w_(1));
    w2 = this->lpf2.update(w_(2));
    w << w0, w1, w2;

    // Вычисление tau_dist
    tau_dist = this->lamb * p_k - w;

    // Вычисление invJ_T
    invJ_T = J.transpose().inverse();

    // Вычисление f_hat
    f_hat = invJ_T * tau_dist;

    // Проверка условия для f_hat[2]
    if (f_hat(2) < 0) {
        f_hat(2) = 0;
    }

    return f_hat;
}