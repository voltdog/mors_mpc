#ifndef STATE_ESTIMATOR_HMB_WORKER_THREADS_HPP
#define STATE_ESTIMATOR_HMB_WORKER_THREADS_HPP

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

namespace state_estimator_hmb
{

inline void StopWorkerThreads(
    std::atomic_bool& running,
    std::condition_variable& wakeup,
    std::mutex& wait_mutex,
    std::vector<std::thread>& threads)
{
    {
        std::lock_guard<std::mutex> lock(wait_mutex);
        running.store(false);
    }
    wakeup.notify_all();
    // Рабочий поток мог уже сбросить running при ошибке. join нужен и тогда.
    for (auto& thread : threads)
    {
        if (thread.joinable())
        {
            thread.join();
        }
    }
}

}  // namespace state_estimator_hmb

#endif  // STATE_ESTIMATOR_HMB_WORKER_THREADS_HPP
