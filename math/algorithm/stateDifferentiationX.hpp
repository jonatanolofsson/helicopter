#pragma once
#ifndef SYS_MATH_STATEDIFFERENTIATIONX_HPP_
#define SYS_MATH_STATEDIFFERENTIATIONX_HPP_

#include <Eigen/Core>
#include <atomic>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <os/exceptions.hpp>

namespace sys {
    namespace math {
        using namespace Eigen;


        template<typename M, typename M::States(*FN)(const typename M::States&, const typename M::States&)>
        struct DifferentiationThreadX {
            typedef DifferentiationThreadX<M,FN> Self;
            static Matrix<typename M::Scalar, M::nofStates, M::nofStates> diff;
            static Matrix<typename M::Scalar, M::nofStates, M::nofStates> dx;
            static const typename M::States* x;
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
                        {
                            std::unique_lock<std::mutex> l(configurationGuard);
                            while(id == lastId && !dying) cond.wait(l);
                            if(dying) throw os::HaltException();
                            lastId = id;
                        }
                        diff.col(n) = FN(*x, (typename M::States)dx.col(n)) / (dx(n,n) * 2);
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

            DifferentiationThreadX()
                : n(counter++)
                , t(&Self::dodediff, this)
            {}

            ~DifferentiationThreadX() {
                dying = true;
                cond.notify_all();
                t.join();
            }

            static Matrix<typename M::Scalar, M::nofStates, M::nofStates>
            differentiate(const typename M::States& x_, const typename M::States& dx_) {
                {
                    std::lock_guard<std::mutex> l(configurationGuard);

                    dx = dx_.asDiagonal();
                    x = &x_;
                    left = M::nofStates;
                    ++id;
                    cond.notify_all();
                }

                std::unique_lock<std::mutex> l(leftGuard);
                while(left) resultCond.wait(l);

                return diff;
            }
        };


        template<typename M, typename M::States(*FN)(const typename M::States&, const typename M::States&)> Matrix<typename M::Scalar, M::nofStates, M::nofStates> DifferentiationThreadX<M,FN>::diff;
        template<typename M, typename M::States(*FN)(const typename M::States&, const typename M::States&)> Matrix<typename M::Scalar, M::nofStates, M::nofStates> DifferentiationThreadX<M,FN>::dx;
        template<typename M, typename M::States(*FN)(const typename M::States&, const typename M::States&)> const typename M::States* DifferentiationThreadX<M,FN>::x;
        template<typename M, typename M::States(*FN)(const typename M::States&, const typename M::States&)> std::atomic<int> DifferentiationThreadX<M,FN>::counter;
        template<typename M, typename M::States(*FN)(const typename M::States&, const typename M::States&)> std::atomic<int> DifferentiationThreadX<M,FN>::left;
        template<typename M, typename M::States(*FN)(const typename M::States&, const typename M::States&)> std::mutex DifferentiationThreadX<M,FN>::leftGuard;
        template<typename M, typename M::States(*FN)(const typename M::States&, const typename M::States&)> std::mutex DifferentiationThreadX<M,FN>::configurationGuard;
        template<typename M, typename M::States(*FN)(const typename M::States&, const typename M::States&)> std::condition_variable DifferentiationThreadX<M,FN>::resultCond;
        template<typename M, typename M::States(*FN)(const typename M::States&, const typename M::States&)> std::condition_variable DifferentiationThreadX<M,FN>::cond;
        template<typename M, typename M::States(*FN)(const typename M::States&, const typename M::States&)> int DifferentiationThreadX<M,FN>::id = 0;
        template<typename M, typename M::States(*FN)(const typename M::States&, const typename M::States&)> bool DifferentiationThreadX<M,FN>::dying = false;


        template<typename M, typename M::States(*FN)(const typename M::States&, const typename M::States&)>
        Matrix<typename M::Scalar, M::nofStates, M::nofStates>
        differentiateStates(const typename M::States& x, const typename M::States& dx) {
            typedef DifferentiationThreadX<M,FN> DThread;
            static DThread threads[M::nofStates];

            return DThread::differentiate(x, dx);
        }

    }
}


#endif
