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

        template<typename M, typename SD, typename ESD>
        struct DifferentiationThread {
            typedef DifferentiationThread<M,SD,ESD> Self;
            static const unsigned J = M::nofStates;
            static const unsigned K = SD::nofStates;
            static Matrix<Scalar, J, K> diff;
            static const typename ESD::StateVector* x;
            static const typename SD::StateVector* dx;
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
                        typename ESD::StateVector xp(*x); xp(SD::template statemap<ESD>(n)) += (*dx)(n);
                        typename ESD::StateVector xm(*x); xm(SD::template statemap<ESD>(n)) -= (*dx)(n);
                        Scalar m = 1.0 / (2.0 * (*dx)(n));
                        diff.col(n) = (M::template predict<ESD>(xp, dT) * m - M::template predict<ESD>(xm, dT) * m);
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
            differentiate(const typename ESD::StateVector& x_, const typename SD::StateVector& dx_, const Scalar dT_) {
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

        template<typename M, typename SD, typename ESD> Matrix<Scalar, DifferentiationThread<M,SD,ESD>::J, DifferentiationThread<M,SD,ESD>::K> DifferentiationThread<M,SD,ESD>::diff;
        template<typename M, typename SD, typename ESD> const typename ESD::StateVector* DifferentiationThread<M,SD,ESD>::x;
        template<typename M, typename SD, typename ESD> const typename SD::StateVector* DifferentiationThread<M,SD,ESD>::dx;
        template<typename M, typename SD, typename ESD> Scalar DifferentiationThread<M,SD,ESD>::dT;
        template<typename M, typename SD, typename ESD> std::atomic<int> DifferentiationThread<M,SD,ESD>::counter;
        template<typename M, typename SD, typename ESD> std::atomic<int> DifferentiationThread<M,SD,ESD>::left;
        template<typename M, typename SD, typename ESD> std::mutex DifferentiationThread<M,SD,ESD>::leftGuard;
        template<typename M, typename SD, typename ESD> std::mutex DifferentiationThread<M,SD,ESD>::runGuard;
        template<typename M, typename SD, typename ESD> std::mutex DifferentiationThread<M,SD,ESD>::configurationGuard;
        template<typename M, typename SD, typename ESD> std::condition_variable DifferentiationThread<M,SD,ESD>::resultCond;
        template<typename M, typename SD, typename ESD> std::condition_variable DifferentiationThread<M,SD,ESD>::cond;
        template<typename M, typename SD, typename ESD> int DifferentiationThread<M,SD,ESD>::id = 0;
        template<typename M, typename SD, typename ESD> bool DifferentiationThread<M,SD,ESD>::dying = false;


        template<typename M, typename SD, typename ESD>
        auto differentiate(const typename ESD::StateVector& x, const typename SD::StateVector& dx, const Scalar dT) -> decltype(DifferentiationThread<M,SD,ESD>::differentiate(x, dx, dT)) {
            typedef DifferentiationThread<M,SD,ESD> DThread;
            #pragma clang diagnostic push
            #pragma clang diagnostic ignored "-Wunused-variable"

            static DThread threads[SD::nofStates];

            #pragma clang diagnostic pop

            return DThread::differentiate(x, dx, dT);
        }

        template<typename M, typename SD, typename ESD>
        auto differentiate(const typename ESD::StateVector& x, const Scalar dT = settings::dT) -> decltype(differentiate<M,SD,ESD>(x,typename SD::StateVector(),dT)) {
            typename SD::StateVector dx = SD::StateVector::Constant(1e-3);
            return differentiate<M,SD,ESD>(x, dx, dT);
        }
    }
}


#endif
