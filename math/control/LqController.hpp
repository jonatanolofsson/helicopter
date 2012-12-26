#define SYS_MATH_LQ_CONTROLLER_HPP_
#define SYS_MATH_LQ_CONTROLLER_HPP_

#include <Eigen/Core>
#include <Eigen/HouseholderQR.h>

namespace sys {
    template<typename Scalar, int number_of_states, int number_of_controls, char CD>
    class LqController {
        private:
            Scalar alpha;
        public:
            typedef Matrix<scalar, number_of_states, number_of_states> StateMatrix; ///< A state (propagation) matrix describes how the model state evolves in time, given no control
            typedef Matrix<scalar, number_of_states, 1> StateVector; ///< A state vector describes the state in which the system is (currently) in
            typedef Matrix<scalar, number_of_controls, 1> ControlVector; ///< A control vector contains information about the controller output
            typedef Matrix<scalar, number_of_controls, number_of_states> FeedbackMatrix; ///< A feedback matrix is used to calculate the optimal control signal, given a current state fo the system, to bring the system to a zero-state
            ControlVector u;
            FeedbackMatrix L;
            StateMatrix A, P, deltaP, Q;
            HouseholderQR Rinv;

            LqController() : alpha(1.0) {
                u.setZero();
                L.setZero();
            };

            void updateModel(const SystemModel m) {
                A = m.A;
                B = m.B;
                updateControlMatrices();
            }

            void updateControlMatrices() {
                if(CD == 'C') {
                    auto BRB = B*Rinv.solve(B.transpose());
                    StateMatrix deltaP;
                    do {
                        deltaP = alpha*(A.transpose()*P + P*A - P*BRB*P + Q).householderQr().solve(P);
                        P += deltaP;
                    } while(deltaP.abs().maxCoeff() > P_DELTA_EPSILON);

                    L = Rinv.solve(B.transpose()*P);
                } else {
                    auto BRB = B*Rinv.solve(B.transpose());
                    StateMatrix Pprev;

                    do {
                        Pprev = P;
                        P = Q + A.transpose()*(P - P*B*(R + B.transpose()*P*B).ldlt().solve(B.transpose()*P)*A;
                    } while((P-Pprev).abs().maxCoeff() > P_DELTA_EPSILON);

                    L = (R + B.transpose()*P*B).ldlt().solve(B.transpose()*P*A);
                }
            }

            /*!
             * \brief   Calculate the control signal optimal for bringing the system to the origin
             * \param   x   Current state of system
             * \return  Optimal control signal, given cost matrices of model
             */
            const ControlVector& control_signal(const StateVector& x) {
                /// [2] eq. 9.11: u = L*x, r = 0
                u = -L*x;
                return u;
            }

            const ControlVector& operator()(const StateVector& x) {
                return control_signal(x);
            }
    };
}
/// [2]: Glad & Ljung, Reglerteori (Studentlitteratur, 2003)

#endif
