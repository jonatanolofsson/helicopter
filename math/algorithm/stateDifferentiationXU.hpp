#pragma once
#ifndef SYS_MATH_STATEDIFFERENTIATIONXU_HPP_
#define SYS_MATH_STATEDIFFERENTIATIONXU_HPP_

#include <Eigen/Core>
#include <atomic>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <os/exceptions.hpp>

namespace sys {
    namespace math {
        using namespace Eigen;


        template<typename M, typename D, typename M::States(*FN)(const typename M::States&, const typename M::Controls&, const D&)>
        struct DifferentiationThreadXU {
            typedef DifferentiationThreadXU<M,D,FN> Self;
            static Matrix<typename M::Scalar, M::States::RowsAtCompileTime, D::RowsAtCompileTime> diff;
            static Matrix<typename M::Scalar, D::RowsAtCompileTime, D::RowsAtCompileTime> dx;
            static const typename M::States* x;
            static const typename M::Controls* u;
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
                        diff.col(n) = FN(*x, *u, (D)dx.col(n)) / (dx(n,n) * 2);
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

            DifferentiationThreadXU()
                : n(counter++)
                , t(&Self::dodediff, this)
            {}

            ~DifferentiationThreadXU() {
                dying = true;
                cond.notify_all();
                t.join();
            }

            static Matrix<typename M::Scalar, M::States::RowsAtCompileTime, D::RowsAtCompileTime>
            differentiate(const typename M::States& x_, const typename M::Controls& u_, const D& dx_) {
                {
                    std::lock_guard<std::mutex> l(configurationGuard);

                    dx = dx_.asDiagonal();
                    x = &x_;
                    u = &u_;
                    left = D::RowsAtCompileTime;
                    ++id;
                    cond.notify_all();
                }

                std::unique_lock<std::mutex> l(leftGuard);
                while(left) resultCond.wait(l);

                return diff;
            }
        };


        template<typename M, typename D, typename M::States(*FN)(const typename M::States&, const typename M::Controls&, const D&)> Matrix<typename M::Scalar, M::States::RowsAtCompileTime, D::RowsAtCompileTime> DifferentiationThreadXU<M,D,FN>::diff;
        template<typename M, typename D, typename M::States(*FN)(const typename M::States&, const typename M::Controls&, const D&)> Matrix<typename M::Scalar, D::RowsAtCompileTime, D::RowsAtCompileTime> DifferentiationThreadXU<M,D,FN>::dx;
        template<typename M, typename D, typename M::States(*FN)(const typename M::States&, const typename M::Controls&, const D&)> const typename M::States* DifferentiationThreadXU<M,D,FN>::x;
        template<typename M, typename D, typename M::States(*FN)(const typename M::States&, const typename M::Controls&, const D&)> const typename M::Controls* DifferentiationThreadXU<M,D,FN>::u;
        template<typename M, typename D, typename M::States(*FN)(const typename M::States&, const typename M::Controls&, const D&)> std::atomic<int> DifferentiationThreadXU<M,D,FN>::counter;
        template<typename M, typename D, typename M::States(*FN)(const typename M::States&, const typename M::Controls&, const D&)> std::atomic<int> DifferentiationThreadXU<M,D,FN>::left;
        template<typename M, typename D, typename M::States(*FN)(const typename M::States&, const typename M::Controls&, const D&)> std::mutex DifferentiationThreadXU<M,D,FN>::leftGuard;
        template<typename M, typename D, typename M::States(*FN)(const typename M::States&, const typename M::Controls&, const D&)> std::mutex DifferentiationThreadXU<M,D,FN>::configurationGuard;
        template<typename M, typename D, typename M::States(*FN)(const typename M::States&, const typename M::Controls&, const D&)> std::condition_variable DifferentiationThreadXU<M,D,FN>::resultCond;
        template<typename M, typename D, typename M::States(*FN)(const typename M::States&, const typename M::Controls&, const D&)> std::condition_variable DifferentiationThreadXU<M,D,FN>::cond;
        template<typename M, typename D, typename M::States(*FN)(const typename M::States&, const typename M::Controls&, const D&)> int DifferentiationThreadXU<M,D,FN>::id = 0;
        template<typename M, typename D, typename M::States(*FN)(const typename M::States&, const typename M::Controls&, const D&)> bool DifferentiationThreadXU<M,D,FN>::dying = false;


        template<typename M, typename D, typename M::States(*FN)(const typename M::States&, const typename M::Controls&, const D&)>
        Matrix<typename M::Scalar, M::States::RowsAtCompileTime, D::RowsAtCompileTime>
        differentiateStates(const typename M::States& x, const typename M::Controls& u, const D& dx) {
            typedef DifferentiationThreadXU<M,D,FN> DThread;
            static DThread threads[D::RowsAtCompileTime];

            return DThread::differentiate(x, u, dx);
        }

    }
}


#endif
