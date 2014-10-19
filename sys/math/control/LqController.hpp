#pragma once
#ifndef SYS_MATH_LQCONTROLLER_HPP_
#define SYS_MATH_LQCONTROLLER_HPP_

#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>
#include <os/utils/eventlog.hpp>
#include <sys/types.hpp>
#include <iostream>


#ifndef P_DELTA_EPSILON
    #define P_DELTA_EPSILON 1e-4
#endif

namespace sys {
    namespace math {
        using namespace Eigen;
        template<typename States_, typename Controls_, bool EXTRA_STATE = false>
        class LqController {
            public:
                typedef States_ States;
                typedef Controls_ Controls;
                typedef LqController<States, Controls> Self;
                static const int nofStates = States::nofStates + (EXTRA_STATE ? 1 : 0);
                static const int nofControls = Controls::nofStates;
                static const int linState = States::nofStates;

                typedef Matrix<Scalar, nofStates, nofStates> StateMatrix; ///< A state propagation matrix describes how the model state evolves in time, given no control
                typedef Matrix<Scalar, nofStates, nofControls> ControlMatrix; ///<
                typedef Matrix<Scalar, nofStates, 1> StateVector; ///< A state vector describes the state in which the system is (currently) in
                typedef Matrix<Scalar, nofControls, 1> ControlVector; ///< A state vector describes the state in which the system is (currently) in
                typedef Matrix<Scalar, nofControls, nofStates> FeedbackMatrix; ///< A feedback matrix is used to calculate the optimal control signal, given a current state fo the system, to bring the system to a zero-state
                typedef Matrix<Scalar, nofControls, nofControls> ControlWeightMatrix;

            private:
                Scalar alpha;

            public:
                ControlVector u;
                FeedbackMatrix L;
                StateMatrix A, P, deltaP, Q;
                ControlMatrix B;
                ControlWeightMatrix R;

                LqController() : alpha(1.0) {
                    A.setZero();
                    B.setZero();
                    u.setZero();
                    L.setZero();
                    Q.setIdentity();
                    R.setIdentity();
                    resetRiccati();
                }

                void resetRiccati() {
                    P = Q;
                }

                template<bool isDiscrete>
                void updateModel(const StateMatrix& A_, const ControlMatrix& B_) {
                    A = A_;
                    //LOG_EVENT(typeid(Self).name(), 50, "A::::::::::::::\n" << A << "\n:::::::::::::::::::::::::::::::::::::");
                    B = B_;
                    //LOG_EVENT(typeid(Self).name(), 50, "B::::::::::::::\n" << B << "\n:::::::::::::::::::::::::::::::::::::");
                    updateControlMatrices<isDiscrete>();
                }

                template<bool isDiscrete>
                void updateControlMatrices() {
                    Scalar measure = 1000;
                    Scalar newMeasure;
                    if(isDiscrete) {
                        auto Rinv = R.householderQr();
                        auto BRB = B*Rinv.solve(B.transpose());
                        StateMatrix deltaP;

                        do {
                            deltaP = alpha*(A.transpose()*P + P*A - P*BRB*P + Q).householderQr().solve(P);
                            P += deltaP;
                            newMeasure = deltaP.cwiseAbs2().maxCoeff();
                            //~ std::cout << "(c) Iteration! (" << newMeasure << ")" << std::endl;
                            if(newMeasure > measure) {
                                //~ std::cout << "(c) Wrong way!" << std::endl;
                                break;
                            }
                            measure = newMeasure;
                        } while(measure > P_DELTA_EPSILON);

                        L = Rinv.solve(B.transpose()*P);
                    } else {
                        StateMatrix Pprev;

                        //~ std::cout << "Eig(A): :::::::::::::::::::::::::::" << std::endl << A.eigenvalues().transpose() << std::endl;

                        do {
                            Pprev = P;
                            P = Q + A.transpose()*(P - P*B*(R + B.transpose()*P*B).ldlt().solve(B.transpose()*P))*A;

                            newMeasure = (P-Pprev).cwiseAbs2().maxCoeff();
                            if(newMeasure > measure) {
                                //~ std::cout << "(d) Wrong way!" << std::endl;
                                break;
                            }
                            measure = newMeasure;
                        } while(measure > P_DELTA_EPSILON);

                        L = (R + B.transpose()*P*B).ldlt().solve(B.transpose()*P*A);
                    }
                    //LOG_EVENT(typeid(Self).name(), 50, "L::::::::::::::\n" << L << "\n:::::::::::::::::::::::::::::::::::::" << std::endl);
                }

                /*!
                 * \brief   Calculate the control signal optimal for bringing the system to the origin
                 * \param   x   Current state of system
                 * \return  Optimal control signal, given cost matrices of model
                 */
                template<typename ExternalStates>
                const ControlVector& eval(const typename ExternalStates::StateVector& x) {
                    /// [2] eq. 9.11: u = L*x, r = 0
                    u = -L * States::template translate<ExternalStates>(x);
                    return u;
                }

                template<typename Derived>
                const ControlVector& direct_eval(const Derived& x) {
                    /// [2] eq. 9.11: u = L*x, r = 0
                    u = -L * x;
                    return u;
                }
        };
    }
}
/// [2]: Glad & Ljung, Reglerteori (Studentlitteratur, 2003)

#endif
