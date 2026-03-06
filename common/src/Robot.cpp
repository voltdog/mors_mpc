#include "Robot.hpp"

Robot::Robot()
{
    ef_frames.resize(4);
    ef_frames[0] = "ef_R1";
    ef_frames[1] = "ef_L1";
    ef_frames[2] = "ef_R2";
    ef_frames[3] = "ef_L2";
    base_frame = "base_link";

    jac_pos_indicies.resize(4);
    jac_pos_indicies[0] = 12;
    jac_pos_indicies[1] = 6;
    jac_pos_indicies[2] = 15;
    jac_pos_indicies[3] = 9;

    dt = 0.002;
}

void Robot::set_timestep(double dt)
{
    this->dt = dt;
}

void Robot::BuildPinocchioModel()
{
    string config_address = mors_sys::GetEnv("CONFIGPATH");
    const std::string urdf_filename = config_address + "/../common/urdf/mors.urdf";

    pinocchio::SE3 config1 = pinocchio::SE3::Identity(); //(0, 0, 0);
    pinocchio::SE3 config2 = pinocchio::SE3::Identity(); //(0, 0, 0);
    // pinocchio::JointModelComposite root_joint;
    // root_joint.addJoint(pinocchio::JointModelTranslation(), config1);
    // root_joint.addJoint(pinocchio::JointModelSphericalZYX(), config2);
    // root_joint.setIndexes(0, 0, 0);
    pinocchio::JointModelFreeFlyer root_joint;

    // std::cout << "root_joint nv: " << root_joint.nv() << std::endl;
    // std::cout << "root_joint nq: " << root_joint.nq() << std::endl;
    // std::cout << "root_joint njoints: " << root_joint.njoints << std::endl;

    pinocchio::urdf::buildModel(urdf_filename, root_joint, model_);
    data_ = std::make_unique<pinocchio::Data>(model_);

    std::cout << "Model name: " << model_.name << std::endl;
    std::cout << "  model nq  = " << model_.nq << " (dimension of configuration space)" << std::endl;
    std::cout << "  model nv  = " << model_.nv << " (dimension of velocity space)" << std::endl;
    std::cout << "  model njoints  = " << model_.njoints << " (dimension of velocity space)" << std::endl;
    std::cout << "  model nframes  = " << model_.nframes << " (dimension of velocity space)" << std::endl;

    // Set model gravity
    model_.gravity.linear() << 0.0, 0.0, -9.81;

    // data_ = pinocchio::Data(model_);
    

    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);
    q(6) = 1.0;

    ComputeForwardKinematics(q, v);
    pinocchio::ccrba(model_, *data_, q, v);
    auto robot_mass = data_->Ig.mass();
    auto robot_inertia = data_->Ig.inertia();

    // std::cout << "Robot mass :" << robot_mass << std::endl;
    // std::cout << "Robot inertia:\n" << robot_inertia << std::endl;

    cout << "Joint positions in the world frame:" << std::endl;
    for (pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model_.njoints; ++joint_id)
        std::cout << std::setw(24) << std::left << model_.names[joint_id] << ": " << std::fixed
                << std::setprecision(2) << data_->oMi[joint_id].translation().transpose() << std::endl;

    // cout << "\nFrame positions in the world frame:" << std::endl;
    // for (pinocchio::FrameIndex frame_id = 0; frame_id < (pinocchio::FrameIndex)model_.nframes; ++frame_id)
    //     std::cout << std::setw(24) << std::left << model_.frames[frame_id].name << ": " << std::fixed
    //             << std::setprecision(4) << data_->oMf[frame_id].translation().transpose() << std::endl;

    base_frame_id = model_.getFrameId(base_frame);
}

void Robot::ComputeForwardKinematics(const VectorXd &q, const VectorXd &v) // size(q) = 18
{
    // Вычисление прямой кинематики
    pinocchio::forwardKinematics(model_, *data_, q, v); // добавить сюда еще и q_dot, если нужно
    
    // Обновление позиций фреймов
    pinocchio::updateFramePlacements(model_, *data_);
}

// Метод для получения позиции стопы
std::vector<Eigen::Vector3d> Robot::GetToePositionsInBaseFrame()
{
    std::vector<Eigen::Vector3d> positions;

    for (const auto& frame_name : ef_frames)
    {
        pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);

        trans_base = data_->oMf[base_frame_id].translation();
        trans_toe = data_->oMf[frame_id].translation();
        toe_position_body_aligned = data_->oMf[base_frame_id].rotation().transpose() * (trans_toe - trans_base);
        positions.push_back(toe_position_body_aligned);
    }
    return positions;
}

std::vector<Eigen::Vector3d> Robot::GetToePositionsInWorldFrame()
{
    std::vector<Eigen::Vector3d> positions;

    for (const auto& frame_name : ef_frames)
    {
        pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);

        trans_toe = data_->oMf[frame_id].translation();
        positions.push_back(trans_toe);
    }
    return positions;
}

std::vector<Eigen::Matrix3d> Robot::GetFootJacobian(const VectorXd &q)
{
    std::vector<Eigen::Matrix3d> jacobians;
    int i = 0;

    Matrix3d R = data_->oMf[base_frame_id].rotation();
    pinocchio::computeJointJacobians(model_, *data_, q);

    for (const auto& frame_name : ef_frames)
    {
        pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);
        Eigen::MatrixXd J_full(6, model_.nv);
        Matrix3d Jac;

        pinocchio::getFrameJacobian(model_, *data_, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_full);
        Jac = R.transpose() * J_full.block(0, jac_pos_indicies[i], 3, 3);
        jacobians.push_back(Jac);

        i++;
    }
    return jacobians;
}

Eigen::Vector3d Robot::getFootVelocityGlobal(int leg_id)
{
    // Get spatial velocity in LOCAL_WORLD_ALIGNED frame
    pinocchio::FrameIndex frame_id = model_.getFrameId(ef_frames[leg_id]);
    Eigen::Vector3d foot_vel_global = pinocchio::getFrameVelocity(model_, *data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED).linear();

    // Eigen::Vector3d foot_pos_base = data_.oMf[leg_id].translation();  // Position of foot in world
    
    // // Compute velocity contribution from base angular motion: ω x r
    // Eigen::Vector3d omega_cross_r = base_angular_velocity_rotated.cross(foot_pos_base);

    // // Compute linear velocity of the foot relative to base
    // Eigen::Vector3d rel_vel = -(foot_vel_global.linear() - omega_cross_r);
    
    // return rel_vel;

    return foot_vel_global;
}

void Robot::ComputeInertiaMatrix(Eigen::VectorXd& q, Eigen::MatrixXd& inertia_matrix)
{
    pinocchio::crba(model_, *data_, q);
    inertia_matrix = data_->M;
}

void Robot::ComputeBiasTerms(Eigen::VectorXd& q, Eigen::VectorXd& q_dot, Eigen::VectorXd& bias_terms)
{
    bias_terms = pinocchio::nonLinearEffects(model_, *data_, q, q_dot);
}

bool Robot::ComputeIK_CLIK(
    Eigen::VectorXd &q,
    const std::vector<Eigen::Vector3d> &desired_pos_base,
    int max_iter,
    double tol,
    double gain,
    double damping)
{
    if (desired_pos_base.size() != 4)
        return false;

    const int task_dim = 12;   // 4 ноги × 3 координаты
    const int dof_dim  = 12;   // 4 ноги × 3 сустава

    std::vector<std::string> ik_ef_frames(4);
    ik_ef_frames[0] = "ef_L1";
    ik_ef_frames[1] = "ef_L2";
    ik_ef_frames[2] = "ef_R1";
    ik_ef_frames[3] = "ef_R2";

    std::vector<int> ik_jac_pos_indicies(4);
    ik_jac_pos_indicies[0] = 6;
    ik_jac_pos_indicies[1] = 9;
    ik_jac_pos_indicies[2] = 12;
    ik_jac_pos_indicies[3] = 15;

    for (int iter = 0; iter < max_iter; ++iter)
    {
        // --- FK ---
        Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
        ComputeForwardKinematics(q, v);

        Eigen::Matrix3d R_base = data_->oMf[base_frame_id].rotation();
        Eigen::Vector3d trans_base = data_->oMf[base_frame_id].translation();

        // --- Ошибка (12×1) ---
        Eigen::VectorXd error(task_dim);
        error.setZero();

        // --- Якобиан (12×12) ---
        Eigen::MatrixXd J(task_dim, dof_dim);
        J.setZero();

        pinocchio::computeJointJacobians(model_, *data_, q);

        for (int i = 0; i < 4; ++i)
        {
            pinocchio::FrameIndex frame_id = model_.getFrameId(ik_ef_frames[i]);

            Eigen::Vector3d trans_toe = data_->oMf[frame_id].translation();

            Eigen::Vector3d current_pos_base =
                R_base.transpose() * (trans_toe - trans_base);

            Eigen::Vector3d err_i =
                desired_pos_base[i] - current_pos_base;

            error.segment<3>(3*i) = err_i;

            // --- Якобиан ноги ---
            Eigen::MatrixXd J_full(6, model_.nv);
            pinocchio::getFrameJacobian(
                model_,
                *data_,
                frame_id,
                pinocchio::LOCAL_WORLD_ALIGNED,
                J_full);

            Eigen::Matrix3d J_leg =
                R_base.transpose() *
                J_full.block(0, ik_jac_pos_indicies[i], 3, 3);

            J.block<3,3>(3*i, 3*i) = J_leg;
        }
        // if (!(iter%10))
        //     cout << iter << ": err = " << error.segment<3>(0).transpose() << endl;
        // --- Проверка сходимости ---
        if (error.norm() < tol)
        {   
            // cout << iter << endl;
            return true;
        }
        // --- Damped Least Squares ---
        Eigen::MatrixXd H = J * J.transpose() + damping * Eigen::MatrixXd::Identity(task_dim, task_dim);

        Eigen::VectorXd dq_legs = gain * J.transpose() * H.inverse() * error;

        // --- Формируем полный dq ---
        Eigen::VectorXd dq = Eigen::VectorXd::Zero(model_.nv);

        for (int i = 0; i < 4; ++i)
            dq.segment(ik_jac_pos_indicies[i], 3) = dq_legs.segment<3>(3*i);

        // // --- Интеграция ---
        q = pinocchio::integrate(model_, q, dq*0.5);
    }

    return false;
}

void Robot::ComputeAllLegDynamics(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& q_dot,
    std::vector<Eigen::Matrix3d>& M_legs,
    std::vector<Eigen::Vector3d>& h_legs)
{
    // M_legs.resize(4);
    // h_legs.resize(4);

    int start_idx = 7; // индекс первого сустава ноги
    std::vector<std::vector<int>> leg_joint_indices(4);
    for (int leg = 0; leg < 4; ++leg) {
        for (int j = 0; j < 3; ++j) {
            leg_joint_indices[leg].push_back(start_idx + leg*3 + j);
        }
    }

    pinocchio::ccrba(model_, *data_, q, q_dot);                     // вычисляет M(q) в data.M
    pinocchio::nonLinearEffects(model_, *data_, q, q_dot); // вычисляет nle = C(q,qdot)*qdot + g(q) в data.nle

    for (int leg = 0; leg < 4; leg++) {
        const auto& idx = leg_joint_indices[leg]; // вектор из трёх индексов

        // Матрица инерции ноги 3x3
        Eigen::Matrix3d M_leg = data_->M.block(idx[0], idx[0], 3, 3);
        M_legs[leg] = M_leg;

        // Вектор нелинейных членов ноги 3x1
        Eigen::Vector3d h_leg = data_->nle.segment(idx[0], 3);
        h_legs[leg] = h_leg;
    }
}