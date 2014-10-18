#pragma once
#ifndef SYS_MATH_MODELS_HELICOPTER_HPP_
#define SYS_MATH_MODELS_HELICOPTER_HPP_

#include <Eigen/Core>
#include <sys/types.hpp>
#include <sys/math/constants.hpp>
#include <sys/math/algorithm.hpp>
#include <sys/settings.hpp>
#include <cmath>

#include <iostream>

namespace sys {
    namespace math {
        namespace models {
            using namespace Eigen;
            
            template<typename ModelDescription_>
            struct HelicopterModel {
                typedef ModelDescription_ ModelDescription;
                typedef HelicopterModel<ModelDescription> Self;
                typedef typename ModelDescription::Scalar Scalar;
                typedef typename ModelDescription::States States;
                typedef typename ModelDescription::Controls Controls;
                typedef typename ModelDescription::StateDescription states;
                typedef typename ModelDescription::ControlDescription controls;
                typedef typename Eigen::Matrix<Scalar, 2, 1> Vector2;
                typedef typename Eigen::Matrix<Scalar, 3, 1> Vector3;
                typedef typename Eigen::Matrix<Scalar, 6, 1> Vector6;
                static const bool isDiscrete = true;
                static const Scalar I_g_helicopter_values[9];

                static const Scalar alpha_tail;
                static const Scalar alpha_main;
                static const Scalar eta_F_tail;
                static const Scalar eta_F_rotor;
                static const Scalar eta_M_tail;
                static const Scalar eta_M_rotor;
                static const Scalar motorConstant;
                static const Scalar mass;
                static const Eigen::Matrix<Scalar, 3, 3> I_g;
                static const Eigen::Matrix<Scalar, 3, 3> I_g_inv;
                static const Vector3 r_rotor;
                static const Vector3 r_tail;
                static const Scalar r_tilt;
                static const Vector2 r_A;
                static const Vector2 r_B;
                static const Vector2 r_C;
                
                template<typename IModelDescription = ModelDescription, typename DerivedFilter, typename DerivedControl>
                static Vector6 accPredition(const DerivedFilter& filter, const DerivedControl& u) {
                    typedef typename IModelDescription::StateDescription states;
                    typedef typename math::models::CHelicopter controls;

                    Eigen::Quaternion<typename DerivedFilter::Scalar> qwb(
                        filter.state(states::qw),
                        filter.state(states::qx),
                        filter.state(states::qy),
                        filter.state(states::qz)
                    );
                    qwb.normalize();

                    auto bf2ned = qwb.toRotationMatrix(); // q^{wb}
                    //auto ned2bf = bf2ned.transpose(); // Orthonormal matrix

                    Vector3 A; A << r_A, -r_tilt*sin(u[controls::th_a]);
                    Vector3 B; B << r_B, -r_tilt*sin(u[controls::th_b]);
                    Vector3 C; C << r_C, r_tilt*sin(u[controls::th_c]);

                    auto AB = B-A;
                    auto AC = C-A;
                    auto abc = AB.cross(AC);
                    auto D = abc.dot(A) / abc.z();

                    Vector6 xdot;
                    auto sin2Th_tail = std::sin(2*u[controls::th_tail]);
                    auto N2 = SQUARE(u[controls::N]);
                    xdot.template segment<3>(states::velocity) = bf2ned * Vector3(0, eta_F_tail * N2 * sin2Th_tail / mass, -eta_F_rotor * D * N2 / mass) + Vector3(0, 0, math::constants::g);
                    
                    static const Scalar cosPhi_adj = 1;
                    static const Scalar sinPhi_adj = 0;
                    xdot.template segment<3>(states::rotationalVelocity) = bf2ned * I_g_inv * (
                            Vector3(abc.x()*sinPhi_adj + abc.y()*cosPhi_adj, abc.x()*sinPhi_adj-abc.y()*cosPhi_adj, 0) * eta_M_rotor / abc.z()
                            - eta_M_rotor * r_rotor.cross(Vector3(0,0,D * N2))
                            + eta_M_tail * r_tail.cross(Vector3(0, N2 * sin2Th_tail, 0))
                        );

                    return xdot;
                }

                template<typename DerivedStates, typename DerivedControl>
                static States controlledPrediction(const DerivedStates& x, const DerivedControl& u, const Scalar dT) {
                    static_assert((int)DerivedStates::RowsAtCompileTime >= (int)States::RowsAtCompileTime, "x is not a superset");
                    States xnext(x.template segment<States::RowsAtCompileTime>(0));
                    USING_XYZ

                    xnext.template segment<3>(states::position) += x.template segment<3>(states::velocity) * dT;

                    const Scalar wnorm = x.template segment<3>(states::rotationalVelocity).norm();

                    if(wnorm > math::constants::EPSILON) {
                        const Scalar wnormT2   = wnorm * dT / 2.0;
                        Matrix<Scalar, 4, 4> S;
                        S << QUATERNION_ROTATION_FROM_ROTVEL(x(states::omega[X]), x(states::omega[Y]), x(states::omega[Z]));

                        xnext.template segment<4>(states::quaternion) =
                            (std::cos(wnormT2) * x.template segment<4>(states::quaternion)
                                - (std::sin(wnormT2) / wnorm) * S * x.template segment<4>(states::quaternion)
                            ).normalized();
                    }

                    xnext.template segment<6>(states::velocities) += accPredition(x, u) * dT;

                    return xnext;
                }

                template<typename IModelDescription = ModelDescription>
                static Eigen::Matrix<Scalar, 5, 1> controlDerivative(const typename IModelDescription::States& x, const typename IModelDescription::Controls& u) {
                    typedef typename IModelDescription::StateDescription states;
                    typedef typename IModelDescription::ControlDescription controls;

                    Eigen::Matrix<Scalar, 5, 1> xdot;
                    xdot << 
                            alpha_main*(u[controls::th_a] - x[states::th_a]),
                            alpha_main*(u[controls::th_b] - x[states::th_b]),
                            alpha_main*(u[controls::th_c] - x[states::th_c]),
                            alpha_tail*(u[controls::th_tail] - x[states::th_tail]),
                            motorConstant*(u[controls::N] - x[states::N]);

                    return xdot;
                }

                static Matrix<Scalar, ModelDescription::nofStates, ModelDescription::nofStates>
                covariance(const Scalar dT) {
                    typedef Matrix<Scalar, ModelDescription::nofStates, ModelDescription::nofStates> RetType;
                    return RetType::Identity() * dT;
                }
            };
            
            template<typename ModelDescription, typename Enable = void>
            struct Helicopter: public HelicopterModel<ModelDescription> {
                typedef Helicopter<ModelDescription, Enable> Self;
                typedef typename ModelDescription::Scalar Scalar;
                typedef typename ModelDescription::States States;
                typedef typename ModelDescription::Controls Controls;
                typedef typename ModelDescription::StateDescription states;
                typedef typename ModelDescription::ControlDescription controls;
                
                static States predict(const States& x, const Scalar dT) {
                    auto xnext = HelicopterModel<ModelDescription>::simulate(x, dT);
                    xnext.template segment<5>(states::th_tail) = dT*HelicopterModel<ModelDescription>::controlDerivative(x, u);
                    return xnext;
                }

                static Matrix<Scalar, ModelDescription::nofStates, ModelDescription::nofControls>
                controlJacobian(const States& x, const Controls& u) {
                    static const Controls du = Controls::Constant(1e-2);
                    return math::template differentiate<ModelDescription, States, Controls, Controls, &Self::diffControls>(x, u, du);
                }
                
                template<typename D>
                static Matrix<Scalar, ModelDescription::nofStates, ModelDescription::nofStates>
                systemJacobian(const States& x, const Controls& u) {
                    static const States dx = States::Constant(1e-2);
                    return math::template differentiateStates<ModelDescription, States, Controls, States, &Self::diffStates>(x, u, dx);
                }
            };
            
            template<typename ModelDescription>
            struct Helicopter<ModelDescription, typename std::enable_if<0 == ModelDescription::nofControls>::type> : public HelicopterModel<ModelDescription> {
                typedef Helicopter<ModelDescription> Self;
                typedef typename ModelDescription::Scalar Scalar;
                typedef typename ModelDescription::States States;
                typedef typename ModelDescription::StateDescription states;
                
                static States predict(const States& x, const Scalar dT) {
                    return HelicopterModel<ModelDescription>::controlledPrediction(x, x.template segment<5>(states::controlled), dT);
                }
                
                static States diffStates(const States& x, const States& dx) {
                    return predict(x + dx, 1e-2) - predict(x - dx, 1e-2);
                }
                
                static Matrix<Scalar, ModelDescription::nofStates, ModelDescription::nofStates>
                systemJacobian(const States& x) {
                    static const States dx = States::Constant(1e-2);
                    return math::template differentiateStates<ModelDescription, States, &Self::diffStates>(x, dx);
                }
            };
            
            template<typename ModelDescription_> const typename HelicopterModel<ModelDescription_>::Scalar HelicopterModel<ModelDescription_>::alpha_tail     = 0.1;
            template<typename ModelDescription_> const typename HelicopterModel<ModelDescription_>::Scalar HelicopterModel<ModelDescription_>::alpha_main     = 0.1;
            template<typename ModelDescription_> const typename HelicopterModel<ModelDescription_>::Scalar HelicopterModel<ModelDescription_>::eta_F_tail     = 0.1;
            template<typename ModelDescription_> const typename HelicopterModel<ModelDescription_>::Scalar HelicopterModel<ModelDescription_>::eta_F_rotor    = 0.1;
            template<typename ModelDescription_> const typename HelicopterModel<ModelDescription_>::Scalar HelicopterModel<ModelDescription_>::eta_M_tail     = 0.1;
            template<typename ModelDescription_> const typename HelicopterModel<ModelDescription_>::Scalar HelicopterModel<ModelDescription_>::eta_M_rotor    = 0.1;
            template<typename ModelDescription_> const typename HelicopterModel<ModelDescription_>::Scalar HelicopterModel<ModelDescription_>::motorConstant  = 0.1;
            template<typename ModelDescription_> const typename HelicopterModel<ModelDescription_>::Scalar HelicopterModel<ModelDescription_>::mass           = 4.0;
            
            template<typename ModelDescription_> const typename HelicopterModel<ModelDescription_>::Scalar HelicopterModel<ModelDescription_>::I_g_helicopter_values[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
            template<typename ModelDescription_> const Eigen::Matrix<typename HelicopterModel<ModelDescription_>::Scalar, 3, 3> HelicopterModel<ModelDescription_>::I_g(I_g_helicopter_values);
            template<typename ModelDescription_> const Eigen::Matrix<typename HelicopterModel<ModelDescription_>::Scalar, 3, 3> HelicopterModel<ModelDescription_>::I_g_inv(I_g.inverse());
            template<typename ModelDescription_> const typename HelicopterModel<ModelDescription_>::Vector3 HelicopterModel<ModelDescription_>::r_rotor(0, 0, -0.2);
            template<typename ModelDescription_> const typename HelicopterModel<ModelDescription_>::Vector3 HelicopterModel<ModelDescription_>::r_tail(-0.75, 0, -0.05);
            
            template<typename ModelDescription_> const typename HelicopterModel<ModelDescription_>::Scalar HelicopterModel<ModelDescription_>::r_tilt         = 0.032;
            template<typename ModelDescription_> const typename HelicopterModel<ModelDescription_>::Vector2 HelicopterModel<ModelDescription_>::r_A(0.02, 0.034641);
            template<typename ModelDescription_> const typename HelicopterModel<ModelDescription_>::Vector2 HelicopterModel<ModelDescription_>::r_B(0.02, -0.034641);
            template<typename ModelDescription_> const typename HelicopterModel<ModelDescription_>::Vector2 HelicopterModel<ModelDescription_>::r_C(-0.04, 0);
        }
    }
}

#endif
