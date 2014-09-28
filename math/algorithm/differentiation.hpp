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
#include <sys/settings.hpp>
#include <iostream>

namespace sys {
    namespace math {
        using namespace Eigen;


        template<typename M, typename S, typename D, typename Scalar>
        struct DifferentiationThread {
            typedef DifferentiationThread<M,S,D,Scalar> Self;
            typedef decltype(M::Model::predict(S(),Scalar())) R;
            static const unsigned J = R::RowsAtCompileTime;
            static const unsigned K = M::nofStates;
            static Matrix<Scalar, J, K> diff;
            static const S* x;
            static const D* dx;
            static Scalar dT;
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
                        S xp(*x); xp(M::statemap(n)) += (*dx)(n);
                        S xm(*x); xm(M::statemap(n)) -= (*dx)(n);
                        Scalar m = 1.0 / (2.0 * (*dx)(n));
                        diff.col(n) = (M::Model::predict(xp, dT) * m - M::Model::predict(xm, dT) * m);
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
            differentiate(const S& x_, const D& dx_, const Scalar dT_) {
                std::lock_guard<std::mutex> r(runGuard);
                std::unique_lock<std::mutex> l(leftGuard);
                {
                    std::lock_guard<std::mutex> l(configurationGuard);

                    x = &x_;
                    dx = &dx_;
                    dT = dT_;
                    left = K;
                    ++id;
                    cond.notify_all();
                }

                while(left) resultCond.wait(l);

                return diff;
            }
        };

        template<typename M, typename S, typename D, typename Scalar> Matrix<Scalar, DifferentiationThread<M,S,D,Scalar>::J, DifferentiationThread<M,S,D,Scalar>::K> DifferentiationThread<M,S,D,Scalar>::diff;
        template<typename M, typename S, typename D, typename Scalar> const S* DifferentiationThread<M,S,D,Scalar>::x;
        template<typename M, typename S, typename D, typename Scalar> const D* DifferentiationThread<M,S,D,Scalar>::dx;
        template<typename M, typename S, typename D, typename Scalar> Scalar DifferentiationThread<M,S,D,Scalar>::dT;
        template<typename M, typename S, typename D, typename Scalar> std::atomic<int> DifferentiationThread<M,S,D,Scalar>::counter;
        template<typename M, typename S, typename D, typename Scalar> std::atomic<int> DifferentiationThread<M,S,D,Scalar>::left;
        template<typename M, typename S, typename D, typename Scalar> std::mutex DifferentiationThread<M,S,D,Scalar>::leftGuard;
        template<typename M, typename S, typename D, typename Scalar> std::mutex DifferentiationThread<M,S,D,Scalar>::runGuard;
        template<typename M, typename S, typename D, typename Scalar> std::mutex DifferentiationThread<M,S,D,Scalar>::configurationGuard;
        template<typename M, typename S, typename D, typename Scalar> std::condition_variable DifferentiationThread<M,S,D,Scalar>::resultCond;
        template<typename M, typename S, typename D, typename Scalar> std::condition_variable DifferentiationThread<M,S,D,Scalar>::cond;
        template<typename M, typename S, typename D, typename Scalar> int DifferentiationThread<M,S,D,Scalar>::id = 0;
        template<typename M, typename S, typename D, typename Scalar> bool DifferentiationThread<M,S,D,Scalar>::dying = false;


        template<typename M, typename Scalar = sys::Scalar, typename S, typename D>
        auto differentiate(const S& x, const D& dx, const Scalar dT) -> decltype(DifferentiationThread<M,S,D,Scalar>::differentiate(x, dx, Scalar())) {
            typedef DifferentiationThread<M,S,D,Scalar> DThread;
            #pragma clang diagnostic push
            #pragma clang diagnostic ignored "-Wunused-variable"

            static DThread threads[M::nofStates];

            #pragma clang diagnostic pop

            return DThread::differentiate(x, dx, dT);
        }
        
        template<typename M, typename Scalar = sys::Scalar, typename S>
        auto differentiate(const S& x, const Scalar dT = settings::dT) -> decltype(differentiate<M,Scalar,S,decltype(M::Model::predict(S(),Scalar()))>(x,decltype(M::Model::predict(S(),Scalar()))(),dT)) {
            typedef decltype(M::Model::predict(S(),Scalar())) D;

            D dx = D::Constant(1e-3);
            return differentiate<M,Scalar,S,D>(x, dx, dT);
        }
    }
}


#endif
