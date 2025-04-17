#ifndef CONTACTSTATE_FSM_HPP
#define CONTACTSTATE_FSM_HPP

#include <vector>

class ContactStateFSM {
public:
    // Константы состояний
    static constexpr int SWING = 0;
    static constexpr int STANCE = 1;
    static constexpr int LATE = 2;

    // Конструктор
    explicit ContactStateFSM(double start_td_detecting);

    // Метод для выполнения шага
    std::vector<int> step(const std::vector<bool>& contact_flag, const std::vector<double>& phi);

private:
    double start_td_detecting; // Параметр для обнаружения начала фазы
    std::vector<int> state;    // Текущие состояния для каждой ноги
    std::vector<double> phi_pre; // Предыдущие значения фаз
};

#endif // CONTACTSTATE_FSM_HPP