#include "LocomotionNode.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <exception>
#include <iostream>
#include <thread>

namespace {

static_assert(std::atomic<bool>::is_always_lock_free);
std::atomic<bool> stop_requested{false};

void request_stop(int) {
    stop_requested.store(true, std::memory_order_relaxed);
}

} // namespace

int main() 
{
    try 
    {
        std::signal(SIGINT, request_stop);
        std::signal(SIGTERM, request_stop);

        using Clock = std::chrono::steady_clock;

        mors::fsm::LocomotionNode node;
        const auto period = node.period();
        auto next_tick = Clock::now();


        while (!stop_requested.load(std::memory_order_relaxed)) 
        {
            node.step();

            next_tick += period;
            const auto now = Clock::now();
            if (next_tick <= now) {
                // Сохраняем сетку тактов при опоздании.
                next_tick += period * ((now - next_tick) / period + 1);
            }
            std::this_thread::sleep_until(next_tick);
        }
        return 0;
    } 
    catch (const std::exception& error) 
    {
        std::cerr << "[LocomotionNode] " << error.what() << '\n';
        return 1;
    }
}
