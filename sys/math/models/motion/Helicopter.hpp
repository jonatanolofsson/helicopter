#pragma once

#include <Eigen/Core>
#include <sys/types.hpp>
#include <sys/math/constants.hpp>
#include <sys/math/algorithm.hpp>
#include <sys/math/states.hpp>
#include <sys/settings.hpp>
#include <cmath>
#include <os/utils/eventlog.hpp>
#include <os/utils/params.hpp>

#include <iostream>

namespace sys {
    namespace math {
        namespace models {
            using namespace Eigen;
            namespace helicopter {
                template<bool A = false>
                struct Parameters {
                    static Scalar eta_F_tail;
                    static Scalar eta_F_rotor;
                    static Scalar eta_M_tail;
                    static Scalar eta_M_rotor;
                    static Scalar mass;

                    static Eigen::Matrix<Scalar, 3, 3> I_g_inv;
                    static Vector3 r_rotor;
                    static Vector3 r_tail;

                    static Scalar r_tilt;
                    static Vector2 r_A;
                    static Vector2 r_B;
                    static Vector2 r_C;

                    static void initializeFromParams() {
                        eta_F_tail     = os::parameters["Helicopter"]["eta_F_tail"].GetDouble();
                        eta_F_rotor    = os::parameters["Helicopter"]["eta_F_rotor"].GetDouble();
                        eta_M_tail     = os::parameters["Helicopter"]["eta_M_tail"].GetDouble();
                        eta_M_rotor    = os::parameters["Helicopter"]["eta_M_rotor"].GetDouble();
                        mass           = os::parameters["Helicopter"]["mass"].GetDouble();

                        Eigen::Matrix<Scalar, 3, 3> I_g;
                        for(int i = 0; i < 3; ++i) {
                            for(int j = 0; j < 3; ++j) {
                                I_g(i, j) = os::parameters["Helicopter"]["Ig"][i*3 + j].GetDouble();
                            }
                        }
                        I_g_inv = I_g.inverse();
                        for(int i = 0; i < 3; ++i) {
                           r_rotor[i] = os::parameters["Helicopter"]["r_rotor"][i].GetDouble();
                        }
                        for(int i = 0; i < 3; ++i) {
                            r_tail[i] = os::parameters["Helicopter"]["r_tail"][i].GetDouble();
                        }

                        r_tilt = os::parameters["Helicopter"]["r_tilt"].GetDouble();
                        for(int i = 0; i < 2; ++i) {
                            r_A[i] = os::parameters["Helicopter"]["r_A"][i].GetDouble();
                        }
                        for(int i = 0; i < 2; ++i) {
                            r_B[i] = os::parameters["Helicopter"]["r_B"][i].GetDouble();
                        }
                        for(int i = 0; i < 2; ++i) {
                            r_C[i] = os::parameters["Helicopter"]["r_C"][i].GetDouble();
                        }
                    }
                };

                template<bool A> Scalar Parameters<A>::eta_F_tail;
                template<bool A> Scalar Parameters<A>::eta_F_rotor;
                template<bool A> Scalar Parameters<A>::eta_M_tail;
                template<bool A> Scalar Parameters<A>::eta_M_rotor;
                template<bool A> Scalar Parameters<A>::mass;

                template<bool A> Eigen::Matrix<Scalar, 3, 3> Parameters<A>::I_g_inv;
                template<bool A> Vector3 Parameters<A>::r_rotor;
                template<bool A> Vector3 Parameters<A>::r_tail;

                template<bool A> Scalar Parameters<A>::r_tilt;
                template<bool A> Vector2 Parameters<A>::r_A;
                template<bool A> Vector2 Parameters<A>::r_B;
                template<bool A> Vector2 Parameters<A>::r_C;

            }

            template<typename States_ = VW_3D>
            struct Helicopter : public Model<Helicopter<States_>, States_, false> {
                typedef Helicopter<States_> Self;
                typedef Model<Self, States_, false> Base;
                using States = typename Base::States;
                using StateVector = typename Base::StateVector;
                using p = helicopter::Parameters<>;
                typedef States states;

                template<typename ExternalStates>
                static StateVector predict(const typename ExternalStates::StateVector& x, const Scalar = settings::dT) {
                    typedef ExternalStates extstates;
                    StateVector xdot; xdot.setZero();

                    Eigen::Quaternion<Scalar> qwb(
                        x(extstates::qw),
                        x(extstates::qx),
                        x(extstates::qy),
                        x(extstates::qz)
                    );
                    qwb.normalize();

                    auto bf2ned = qwb.toRotationMatrix();

                    Vector3 A; A << p::r_A, -p::r_tilt*sin(x[extstates::th_a]);
                    Vector3 B; B << p::r_B, -p::r_tilt*sin(x[extstates::th_b]);
                    Vector3 C; C << p::r_C,  p::r_tilt*sin(x[extstates::th_c]);
                    std::cout << "A: " << A.transpose() << std::endl;
                    std::cout << "B: " << B.transpose() << std::endl;
                    std::cout << "C: " << C.transpose() << std::endl;

                    auto AB = B-A;
                    auto AC = C-A;
                    auto abc = AB.cross(AC);
                    auto D = abc.dot(A) / abc.z();

                    std::cout << "abc: " << abc.transpose() << std::endl;
                    std::cout << "D: " << D << std::endl;

                    auto sin2Th_tail = std::sin(2*x[extstates::th_tail]);
                    auto N2 = SQUARE(x[extstates::N]);
                    xdot.template segment<3>(states::velocity) = \
                        bf2ned * Vector3(0,
                                         p::eta_F_tail * N2 * sin2Th_tail / p::mass,
                                         -p::eta_F_rotor * D * N2 / p::mass)
                        + Vector3(0, 0, math::constants::g);

                    static const Scalar cosPhi_adj = 1;
                    static const Scalar sinPhi_adj = 0;
                    xdot.template segment<3>(states::omega) = bf2ned * p::I_g_inv * (
                            Vector3(abc.x()*sinPhi_adj + abc.y()*cosPhi_adj,
                                    abc.x()*sinPhi_adj-abc.y()*cosPhi_adj,
                                    0) * p::eta_M_rotor / abc.z()
                            - p::eta_M_rotor * p::r_rotor.cross(Vector3(0,0,D * N2))
                            + p::eta_M_tail * p::r_tail.cross(Vector3(0, N2 * sin2Th_tail, 0))
                        );
                    //LOG_EVENT(typeid(Self).name(), 50, "xdot: " << xdot.transpose());

                    return xdot;
                }
            };

            struct HelicopterImu : public Model<HelicopterImu, VW_3D> {
                typedef HelicopterImu Self;
                typedef Helicopter<VW_3D> Heli;
                typedef AW_3D States;
                using StateVector = typename States::StateVector;
                typedef States states;

                template<typename ExternalStates>
                static StateVector predict(const typename ExternalStates::StateVector& x, const Scalar dT = settings::dT) {
                    auto R = Heli::predict<ExternalStates>(x, dT);
                    R.template segment<3>(states::omega) *= dT;
                    return R;
                }
            };

            template<typename States_ = VWXQ_3D>
            struct HelicopterMotion : public Model<HelicopterMotion<States_>, States_, false> {
                typedef HelicopterMotion<States_> Self;
                typedef Model<Self, States_, false> Base;
                using States = typename Base::States;
                using StateVector = typename Base::StateVector;
                typedef States states;

                template<typename ExternalStates>
                static StateVector predict(const typename ExternalStates::StateVector& x, const Scalar dT = settings::dT) {
                    typedef ExternalStates extstates;
                    StateVector xnext = States::template translateFrom<ExternalStates>(x);

                    xnext.template segment<3>(states::position) += x.template segment<3>(extstates::velocity) * dT;

                    const Scalar wnorm = x.template segment<3>(extstates::omega).norm();

                    if(wnorm > math::constants::EPSILON) {
                        const Scalar wnormT2   = wnorm * dT / 2.0;
                        Matrix<Scalar, 4, 4> S;
                        S << QUATERNION_ROTATION_FROM_ROTVEL(x(extstates::wx), x(extstates::wy), x(extstates::wz));

                        xnext.template segment<4>(states::quaternion) =
                            (std::cos(wnormT2) * x.template segment<4>(states::quaternion)
                                - (std::sin(wnormT2) / wnorm) * S * x.template segment<4>(states::quaternion)
                            ).normalized();
                    }

                    xnext.template segment<6>(states::velocity) += Helicopter<>::predict<ExternalStates>(x) * dT;

                    return xnext;
                }
            };
        }
    }
}

