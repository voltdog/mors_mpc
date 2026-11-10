#include "worker_threads.hpp"

#include <iostream>
#include <mutex>

int main()
{
    for (const bool failed_before_stop : {false, true})
    {
        std::atomic_bool running{true};
        std::atomic_bool exited{false};
        std::condition_variable wakeup;
        std::mutex mutex;
        std::vector<std::thread> threads;
        threads.emplace_back([&]
        {
            std::unique_lock<std::mutex> lock(mutex);
            wakeup.wait(lock, [&] { return !running.load(); });
            exited.store(true);
        });

        // Так рабочий поток сигнализирует фатальную ошибку до вызова Stop().
        if (failed_before_stop)
        {
            std::lock_guard<std::mutex> lock(mutex);
            running.store(false);
        }
        state_estimator_hmb::StopWorkerThreads(running, wakeup, mutex, threads);
        if (!exited.load() || threads.front().joinable())
        {
            std::cerr << "worker_threads_test: worker was not joined\n";
            return 1;
        }
        // Повторная остановка также безопасна.
        state_estimator_hmb::StopWorkerThreads(running, wakeup, mutex, threads);
    }
    std::cout << "worker_threads_test passed\n";
    return 0;
}
