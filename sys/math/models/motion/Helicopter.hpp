#pragma once

#include <Eigen/Core>
#include <sys/types.hpp>
#include <sys/math/constants.hpp>
#include <sys/math/algorithm.hpp>
#include <sys/math/states.hpp>
#include <sys/settings.hpp>
#include <cmath>

#include <iostream>

namespace sys {
    namespace math {
        namespace models {
            using namespace Eigen;
            template<typename States_ = VW_3D>
            struct Helicopter : public Model<Helicopter<States_>, States_, false> {
                typedef Helicopter<States_> Self;
                typedef Model<Self, States_, false> Base;
                using States = typename Base::States;
                using StateVector = typename Base::StateVector;
                typedef States states;

                template<typename ExternalStates>
                static StateVector predict(const typename ExternalStates::StateVector& x, const Scalar = settings::dT) {
                    static const Scalar eta_F_tail     = 0.1;
                    static const Scalar eta_F_rotor    = 0.1;
                    static const Scalar eta_M_tail     = 0.1;
                    static const Scalar eta_M_rotor    = 0.1;
                    static const Scalar mass           = 4.0;

                    static const Scalar I_g_values[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
                    static const Eigen::Matrix<Scalar, 3, 3> I_g(I_g_values);
                    static const Eigen::Matrix<Scalar, 3, 3> I_g_inv(I_g.inverse());
                    static const Vector3 r_rotor(0, 0, -0.2);
                    static const Vector3 r_tail(-0.75, 0, -0.05);

                    static const Scalar r_tilt = 0.032;
                    static const Vector2 r_A(0.02, 0.034641);
                    static const Vector2 r_B(0.02, -0.034641);
                    static const Vector2 r_C(-0.04, 0);

                    typedef ExternalStates extstates;
                    StateVector xdot;

                    Eigen::Quaternion<Scalar> qwb(
                        x(extstates::qw),
                        x(extstates::qx),
                        x(extstates::qy),
                        x(extstates::qz)
                    );
                    qwb.normalize();

                    auto bf2ned = qwb.toRotationMatrix(); // q^{wb}
                    //auto ned2bf = bf2ned.transpose(); // Orthonormal matrix

                    Vector3 A; A << r_A, -r_tilt*sin(x[extstates::th_a]);
                    Vector3 B; B << r_B, -r_tilt*sin(x[extstates::th_b]);
                    Vector3 C; C << r_C,  r_tilt*sin(x[extstates::th_c]);

                    auto AB = B-A;
                    auto AC = C-A;
                    auto abc = AB.cross(AC);
                    auto D = abc.dot(A) / abc.z();

                    auto sin2Th_tail = std::sin(2*x[extstates::th_tail]);
                    auto N2 = SQUARE(x[extstates::N]);
                    xdot.template segment<3>(states::velocity) = bf2ned * Vector3(0, eta_F_tail * N2 * sin2Th_tail / mass, -eta_F_rotor * D * N2 / mass) + Vector3(0, 0, math::constants::g);

                    static const Scalar cosPhi_adj = 1;
                    static const Scalar sinPhi_adj = 0;
                    xdot.template segment<3>(states::omega) = bf2ned * I_g_inv * (
                            Vector3(abc.x()*sinPhi_adj + abc.y()*cosPhi_adj, abc.x()*sinPhi_adj-abc.y()*cosPhi_adj, 0) * eta_M_rotor / abc.z()
                            - eta_M_rotor * r_rotor.cross(Vector3(0,0,D * N2))
                            + eta_M_tail * r_tail.cross(Vector3(0, N2 * sin2Th_tail, 0))
                        );

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
                    StateVector xnext = States::template translate<ExternalStates>(x);

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

