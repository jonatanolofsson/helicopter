#pragma once
#ifndef SYS_MATH_CONTROLIFFERENTIATION_HPP_
#define SYS_MATH_CONTROLIFFERENTIATION_HPP_

#include <Eigen/Core>
#include <atomic>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <os/exceptions.hpp>
#include <iostream>

namespace sys {
    namespace math {
        using namespace Eigen;


        template<typename T, typename T::Controls(*FN)(const T&)>
        struct ControlDifferentiationThread {
            static Matrix<typename T::Controls::Scalar, T::Controls::RowsAtCompileTime, T::Controls::RowsAtCompileTime> diff;
            static Matrix<typename T::Controls::Scalar, T::Controls::RowsAtCompileTime, T::Controls::RowsAtCompileTime> dx;
            static const T* filter;
            static int id;
            static std::atomic<int> counter;
            static std::atomic<int> left;
            static std::mutex configurationGuard, leftGuard;
            static bool dying;
            static std::condition_variable resultCond;
            static std::condition_variable cond;
            int n;
            std::thread t;

            void dodediff() {
                try {
                    int lastId = 0;
                    while(!dying) {
                        std::unique_lock<std::mutex> l(configurationGuard);
                        while(id == lastId && !dying) cond.wait(l);
                        if(dying) throw os::HaltException();
                        lastId = id;
                        l.unlock();
                        diff.col(n) = (FN(*filter+dx.col(n)) - FN(*filter-dx.col(n))) / (dx(n,n) * 2);
                        --left;
                        if(left == 0) {
                            resultCond.notify_all();
                        }
                    }
                }
                catch (os::HaltException& e) {}
                catch (std::exception& e) {
                    std::cerr << e.what();
                }
            }

            StateDifferentiationThread()
                : n(counter++)
                , t(&StateDifferentiationThread<T,FN>::dodediff, this)
            {}

            ~StateDifferentiationThread() {
                dying = true;
                cond.notify_all();
                t.join();
            }
        };


        template<typename T, typename T::Controls(*FN)(const T&)> Matrix<typename T::Controls::Scalar, T::Controls::RowsAtCompileTime, T::Controls::RowsAtCompileTime> StateDifferentiationThread<T,FN>::diff;
        template<typename T, typename T::Controls(*FN)(const T&)> Matrix<typename T::Controls::Scalar, T::Controls::RowsAtCompileTime, T::Controls::RowsAtCompileTime> StateDifferentiationThread<T,FN>::dx;
        template<typename T, typename T::Controls(*FN)(const T&)> const T* StateDifferentiationThread<T,FN>::filter;
        template<typename T, typename T::Controls(*FN)(const T&)> std::atomic<int> StateDifferentiationThread<T,FN>::counter;
        template<typename T, typename T::Controls(*FN)(const T&)> std::atomic<int> StateDifferentiationThread<T,FN>::left;
        template<typename T, typename T::Controls(*FN)(const T&)> std::mutex StateDifferentiationThread<T,FN>::leftGuard;
        template<typename T, typename T::Controls(*FN)(const T&)> std::mutex StateDifferentiationThread<T,FN>::configurationGuard;
        template<typename T, typename T::Controls(*FN)(const T&)> std::condition_variable StateDifferentiationThread<T,FN>::resultCond;
        template<typename T, typename T::Controls(*FN)(const T&)> std::condition_variable StateDifferentiationThread<T,FN>::cond;
        template<typename T, typename T::Controls(*FN)(const T&)> int StateDifferentiationThread<T,FN>::id = 0;
        template<typename T, typename T::Controls(*FN)(const T&)> bool StateDifferentiationThread<T,FN>::dying = false;


        template<typename T, typename T::Controls(*FN)(const T&)>
        Matrix<typename T::Controls::Scalar, T::Controls::RowsAtCompileTime, T::Controls::RowsAtCompileTime> differentiateStates(const T& filter, const typename T::Controls& dx) {
            typedef StateDifferentiationThread<T,FN> DThread;
            static DThread threads[T::Controls::RowsAtCompileTime];

            {
                std::lock_guard<std::mutex> l(DThread::configurationGuard);

                DThread::dx = dx.asDiagonal();
                DThread::filter = &filter;
                DThread::left = T::Controls::RowsAtCompileTime;
                ++DThread::id;
                DThread::cond.notify_all();
            }

            std::unique_lock<std::mutex> l(DThread::leftGuard);
            while(DThread::left) DThread::resultCond.wait(l);
            return DThread::diff;
        }
    }
}


#endif
