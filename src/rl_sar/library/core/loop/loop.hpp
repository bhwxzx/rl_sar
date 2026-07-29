/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LOOP_H
#define LOOP_H

#include <iostream>
#include <thread>
#include <chrono>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <exception>
#include <stdexcept>
#include <utility>
#include <vector>
#include <sstream>
#include <iomanip>
#include "logger.hpp"

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#endif

class LoopFunc
{
public:
    using ErrorCallback = std::function<void(const std::string&, std::exception_ptr)>;

    LoopFunc(
        const std::string &name,
        float period,
        std::function<void()> func,
        int bindCPU = -1,
        ErrorCallback errorCallback = nullptr)
        : _name(name),
          _period(period),
          _func(std::move(func)),
          _bindCPU(bindCPU),
          _errorCallback(std::move(errorCallback)),
          _running(false)
    {
    }

    ~LoopFunc()
    {
        shutdown();
    }

    LoopFunc(const LoopFunc&) = delete;
    LoopFunc& operator=(const LoopFunc&) = delete;

    void start()
    {
        std::lock_guard<std::mutex> lifecycleLock(_lifecycleMutex);
        if (_thread.joinable() || _running.load(std::memory_order_acquire))
        {
            throw std::logic_error("Loop '" + _name + "' is already running");
        }

        _running.store(true, std::memory_order_release);
        std::cout << LOGGER::INFO << "[Loop] Loop start - name: " << _name << ", period: " << formatPeriod() << "ms"
                  << (_bindCPU != -1 ? ", cpu: " + std::to_string(_bindCPU) : ", cpu: unspecified") << std::endl;

        try
        {
            _thread = std::thread(&LoopFunc::loop, this);
            if (_bindCPU != -1)
            {
                setThreadAffinity(_thread.native_handle(), _bindCPU);
            }
        }
        catch (...)
        {
            _running.store(false, std::memory_order_release);
            _cv.notify_all();
            if (_thread.joinable())
            {
                _thread.join();
            }
            throw;
        }
    }

    void shutdown() noexcept
    {
        std::lock_guard<std::mutex> lifecycleLock(_lifecycleMutex);
        const bool hadThread = _thread.joinable();

        _running.store(false, std::memory_order_release);
        _cv.notify_all();

        if (hadThread)
        {
            if (_thread.get_id() == std::this_thread::get_id())
            {
                std::cerr << LOGGER::ERROR << "[Loop] Loop cannot join itself - name: " << _name << std::endl;
                std::terminate();
            }
            _thread.join();
            std::cout << LOGGER::INFO << "[Loop] Loop end - name: " << _name << std::endl;
        }
    }

private:
    std::string _name;
    float _period;
    std::function<void()> _func;
    int _bindCPU;
    ErrorCallback _errorCallback;
    std::atomic<bool> _running;
    std::mutex _mutex;
    std::mutex _lifecycleMutex;
    std::condition_variable _cv;
    std::thread _thread;

    void loop() noexcept
    {
        try
        {
            while (_running.load(std::memory_order_acquire))
            {
                auto start = std::chrono::steady_clock::now();

                _func();

                auto end = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                auto sleepTime = std::chrono::milliseconds(static_cast<int>((_period * 1000) - elapsed.count()));
                if (sleepTime.count() > 0)
                {
                    std::unique_lock<std::mutex> lock(_mutex);
                    if (_cv.wait_for(lock, sleepTime, [this]
                                     { return !_running.load(std::memory_order_acquire); }))
                    {
                        break;
                    }
                }
            }
        }
        catch (...)
        {
            _running.store(false, std::memory_order_release);
            _cv.notify_all();
            const std::exception_ptr error = std::current_exception();

            if (_errorCallback)
            {
                try
                {
                    _errorCallback(_name, error);
                }
                catch (const std::exception& callbackError)
                {
                    std::cerr << LOGGER::ERROR << "[Loop] Error callback failed for '" << _name
                              << "': " << callbackError.what() << std::endl;
                }
                catch (...)
                {
                    std::cerr << LOGGER::ERROR << "[Loop] Error callback failed for '" << _name
                              << "' with an unknown exception" << std::endl;
                }
            }
            else
            {
                std::cerr << LOGGER::ERROR << "[Loop] Unhandled callback exception - name: " << _name << std::endl;
            }
        }
    }

    std::string formatPeriod() const
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(0) << _period * 1000;
        return stream.str();
    }

    void setThreadAffinity(std::thread::native_handle_type threadHandle, int cpuId)
    {
#ifdef __linux__
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpuId, &cpuset);
        if (pthread_setaffinity_np(threadHandle, sizeof(cpu_set_t), &cpuset) != 0)
        {
            std::ostringstream oss;
            oss << "Error setting thread affinity: CPU " << cpuId << " may not be valid or accessible.";
            throw std::runtime_error(oss.str());
        }
#else
        // Thread affinity not supported on this platform
        std::cout << LOGGER::WARNING << "Thread affinity not supported on this platform" << std::endl;
#endif
    }
};

#endif // LOOP_H
