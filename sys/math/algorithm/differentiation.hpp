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


        template<typename SD, typename ESD>
        struct DifferentiationThread {
            typedef DifferentiationThread<SD,ESD> Self;
            typedef decltype(SD::Model::predict<SD>(S()())) R;
            static const unsigned J = R::RowsAtCompileTime;
            static const unsigned K = SD::nofStates;
            static Matrix<Scalar, J, K> diff;
            static const ESD::StateVector* x;
            static const SD::StateVector* dx;
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
                        S xp(*x); xp(SD::statemap<ESD>(n)) += (*dx)(n);
                        S xm(*x); xm(SD::statemap<ESD>(n)) -= (*dx)(n);
                        Scalar m = 1.0 / (2.0 * (*dx)(n));
                        diff.col(n) = (SD::Model::predict<ESD>(xp, dT) * m - SD::Model::predict<ESD>(xm, dT) * m);
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

        template<typename SD, typename ESD> Matrix<Scalar, DifferentiationThread<SD,S,D>::J, DifferentiationThread<SD,S,D>::K> DifferentiationThread<SD,S,D>::diff;
        template<typename SD, typename ESD> const S* DifferentiationThread<SD,S,D>::x;
        template<typename SD, typename ESD> const D* DifferentiationThread<SD,S,D>::dx;
        template<typename SD, typename ESD> Scalar DifferentiationThread<SD,S,D>::dT;
        template<typename SD, typename ESD> std::atomic<int> DifferentiationThread<SD,S,D>::counter;
        template<typename SD, typename ESD> std::atomic<int> DifferentiationThread<SD,S,D>::left;
        template<typename SD, typename ESD> std::mutex DifferentiationThread<SD,S,D>::leftGuard;
        template<typename SD, typename ESD> std::mutex DifferentiationThread<SD,S,D>::runGuard;
        template<typename SD, typename ESD> std::mutex DifferentiationThread<SD,S,D>::configurationGuard;
        template<typename SD, typename ESD> std::condition_variable DifferentiationThread<SD,S,D>::resultCond;
        template<typename SD, typename ESD> std::condition_variable DifferentiationThread<SD,S,D>::cond;
        template<typename SD, typename ESD> int DifferentiationThread<SD,S,D>::id = 0;
        template<typename SD, typename ESD> bool DifferentiationThread<SD,S,D>::dying = false;


        template<typename SD, typename ESD>
        auto differentiate(const ESD::StateVector& x, const SD::StateVector& dx, const Scalar dT) -> decltype(DifferentiationThread<SD,ESD>::differentiate(x, dx, Scalar())) {
            typedef DifferentiationThread<SD,ESD> DThread;
            #pragma clang diagnostic push
            #pragma clang diagnostic ignored "-Wunused-variable"

            static DThread threads[SD::nofStates];

            #pragma clang diagnostic pop

            return DThread::differentiate(x, dx, dT);
        }

        template<typename SD, typename ESD>
        auto differentiate(const ESD::StateVector& x, const Scalar dT = settings::dT) -> decltype(differentiate<SD,S,decltype(SD::Model::predict<SD>(S()()))>(x,decltype(SD::Model::predict<SD>(S()()))(),dT)) {
            SD::StateVector dx = SD::StateVector::Constant(1e-3);
            return differentiate<SD,ESD>(x, dx, dT);
        }
    }
}


#endif
