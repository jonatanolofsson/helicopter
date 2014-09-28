#pragma once
#ifndef SYS_MATH_DIFFERENTIATION_HPP_
#define SYS_MATH_DIFFERENTIATION_HPP_

#include <Eigen/Core>
#include <atomic>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <os/exceptions.hpp>
#include <sys/math/base.hpp>
#include <iostream>

namespace sys {
    namespace math {
        using namespace Eigen;


        template<typename M, typename S, typename D>
        struct DifferentiationThread {
            typedef DifferentiationThread<M,S,D> Self;
            typedef decltype(M::Model::predict(S())) R;
            typedef typename R::Scalar Scalar;
            static const unsigned J = R::RowsAtCompileTime;
            static const unsigned K = countof(M::statemap);
            static Matrix<Scalar, J, K> diff;
            static const S* x;
            static const D* dx;
            static int id;
            static std::atomic<int> counter;
            static std::atomic<int> left;
            static std::mutex configurationGuard, leftGuard, runGuard;
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
                        S xp(*x); xp(M::statemap[n]) += (*dx)(n);
                        S xm(*x); xm(M::statemap[n]) -= (*dx)(n);
                        Scalar m = 1.0 / (2.0 * (*dx)(n));
                        diff.col(n) = (M::Model::predict(xp) * m - M::Model::predict(xm) * m);
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
                , t(&Self::dodediff, this)
            {}

            ~DifferentiationThread() {
                dying = true;
                cond.notify_all();
                t.join();
            }

            static Matrix<Scalar, J, K>
            differentiate(const S& x_, const D& dx_) {
                std::lock_guard<std::mutex> r(runGuard);
                std::unique_lock<std::mutex> l(leftGuard);
                {
                    std::lock_guard<std::mutex> l(configurationGuard);

                    x = &x_;
                    dx = &dx_;
                    left = K;
                    ++id;
                    cond.notify_all();
                }

                while(left) resultCond.wait(l);

                return diff;
            }
        };

        template<typename M, typename S, typename D> Matrix<typename DifferentiationThread<M,S,D>::R::Scalar, DifferentiationThread<M,S,D>::J, DifferentiationThread<M,S,D>::K> DifferentiationThread<M,S,D>::diff;
        template<typename M, typename S, typename D> const S* DifferentiationThread<M,S,D>::x;
        template<typename M, typename S, typename D> const D* DifferentiationThread<M,S,D>::dx;
        template<typename M, typename S, typename D> std::atomic<int> DifferentiationThread<M,S,D>::counter;
        template<typename M, typename S, typename D> std::atomic<int> DifferentiationThread<M,S,D>::left;
        template<typename M, typename S, typename D> std::mutex DifferentiationThread<M,S,D>::leftGuard;
        template<typename M, typename S, typename D> std::mutex DifferentiationThread<M,S,D>::runGuard;
        template<typename M, typename S, typename D> std::mutex DifferentiationThread<M,S,D>::configurationGuard;
        template<typename M, typename S, typename D> std::condition_variable DifferentiationThread<M,S,D>::resultCond;
        template<typename M, typename S, typename D> std::condition_variable DifferentiationThread<M,S,D>::cond;
        template<typename M, typename S, typename D> int DifferentiationThread<M,S,D>::id = 0;
        template<typename M, typename S, typename D> bool DifferentiationThread<M,S,D>::dying = false;


        template<typename M, typename S, typename D>
        auto differentiate(const S& x, const D& dx) -> decltype(DifferentiationThread<M,S,D>::differentiate(x, dx)) {
            typedef DifferentiationThread<M,S,D> DThread;
            #pragma clang diagnostic push
            #pragma clang diagnostic ignored "-Wunused-variable"

            static DThread threads[countof(M::statemap)];

            #pragma clang diagnostic pop

            return DThread::differentiate(x, dx);
        }
        
        template<typename M, typename S>
        auto differentiate(const S& x) -> decltype(differentiate<M,S,decltype(M::Model::predict(S()))>(x,decltype(M::Model::predict(S()))())) {
            typedef decltype(M::Model::predict(S())) D;

            D dx = D::Constant(1e-3);
            return differentiate<M,S,D>(x, dx);
        }
    }
}


#endif
