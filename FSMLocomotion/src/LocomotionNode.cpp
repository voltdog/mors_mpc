#include "LocomotionNode.hpp"

LocomotionNode::LocomotionNode() {
    // Constructor implementation

    // Запустить поток LocomotionControl.start_thread()

    // инициализировать LCMExchanger и запустить его поток start_exchanger()

    // инициализировать RobotStatus и ReadinessChecker

    // инициализировать все необходимое для работы RobotFSM


}

void LocomotionNode::step() {
    // прочитать данные из LCM

    // обновить состояние RobotFSM

    // задать команды для LocomotionControl

    // отправить данные в LCM
}