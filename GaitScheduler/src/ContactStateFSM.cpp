#include "ContactStateFSM.hpp"

// Конструктор
ContactStateFSM::ContactStateFSM(double start_td_detecting)
    : start_td_detecting(start_td_detecting), state(4, STANCE), phi_pre(4, 0.0) {}

// Метод step
std::vector<int> ContactStateFSM::step(const std::vector<bool>& contact_flag, const std::vector<double>& phi) {
    for (int i = 0; i < 4; ++i) {
        if (state[i] == SWING) {
            if (contact_flag[i] && phi[i] > start_td_detecting) {
                state[i] = STANCE;
            }
            if (phi[i] < 0 && phi_pre[i] > 0) {
                state[i] = LATE;
            }
        } else if (state[i] == STANCE) {
            if (phi[i] >= 0 && phi_pre[i] < 0) {
                state[i] = SWING;
            }
        } else if (state[i] == LATE) {
            if (contact_flag[i]) {
                state[i] = STANCE;
            }
        }

        // Обновляем предыдущее значение фазы
        phi_pre[i] = phi[i];
    }

    return state;
}