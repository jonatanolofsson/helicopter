#pragma once
#ifndef SYS_MATH_STATEDIFFERENTIATION_HPP_
#define SYS_MATH_STATEDIFFERENTIATION_HPP_

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


        template<typename T, typename D, typename T::States(*FN)(const T&, const D&)>
        struct DifferentiationThread {
            static Matrix<typename D::Scalar, T::States::RowsAtCompileTime, D::RowsAtCompileTime> diff;
            static Matrix<typename D::Scalar, D::RowsAtCompileTime, D::RowsAtCompileTime> dx;
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
                        diff.col(n) = FN(*filter, (D)dx.col(n)) / (dx(n,n) * 2);
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

            DifferentiationThread()
                : n(counter++)
                , t(&DifferentiationThread<T,D,FN>::dodediff, this)
            {}

            ~DifferentiationThread() {
                dying = true;
                cond.notify_all();
                t.join();
            }

            static Matrix<typename T::States::Scalar, T::States::RowsAtCompileTime, D::RowsAtCompileTime>
            differentiate(const T& filter_, const D& dx_) {
                {
                    std::lock_guard<std::mutex> l(configurationGuard);

                    dx = dx_.asDiagonal();
                    filter = &filter_;
                    left = D::RowsAtCompileTime;
                    ++id;
                    cond.notify_all();
                }

                std::unique_lock<std::mutex> l(leftGuard);
                while(left) resultCond.wait(l);

                return diff;
            }
        };


        template<typename T, typename D, typename T::States(*FN)(const T&, const D&)> Matrix<typename D::Scalar, T::States::RowsAtCompileTime, D::RowsAtCompileTime> DifferentiationThread<T,D,FN>::diff;
        template<typename T, typename D, typename T::States(*FN)(const T&, const D&)> Matrix<typename D::Scalar, D::RowsAtCompileTime, D::RowsAtCompileTime> DifferentiationThread<T,D,FN>::dx;
        template<typename T, typename D, typename T::States(*FN)(const T&, const D&)> const T* DifferentiationThread<T,D,FN>::filter;
        template<typename T, typename D, typename T::States(*FN)(const T&, const D&)> std::atomic<int> DifferentiationThread<T,D,FN>::counter;
        template<typename T, typename D, typename T::States(*FN)(const T&, const D&)> std::atomic<int> DifferentiationThread<T,D,FN>::left;
        template<typename T, typename D, typename T::States(*FN)(const T&, const D&)> std::mutex DifferentiationThread<T,D,FN>::leftGuard;
        template<typename T, typename D, typename T::States(*FN)(const T&, const D&)> std::mutex DifferentiationThread<T,D,FN>::configurationGuard;
        template<typename T, typename D, typename T::States(*FN)(const T&, const D&)> std::condition_variable DifferentiationThread<T,D,FN>::resultCond;
        template<typename T, typename D, typename T::States(*FN)(const T&, const D&)> std::condition_variable DifferentiationThread<T,D,FN>::cond;
        template<typename T, typename D, typename T::States(*FN)(const T&, const D&)> int DifferentiationThread<T,D,FN>::id = 0;
        template<typename T, typename D, typename T::States(*FN)(const T&, const D&)> bool DifferentiationThread<T,D,FN>::dying = false;


        template<typename T, typename D, typename T::States(*FN)(const T&, const D&)>
        Matrix<typename T::States::Scalar, T::States::RowsAtCompileTime, D::RowsAtCompileTime>
        differentiateStates(const T& filter, const D& dx) {
            typedef DifferentiationThread<T,D,FN> DThread;
            static DThread threads[D::RowsAtCompileTime];

            return DThread::differentiate(filter, dx);
        }

    }
}


#endif
