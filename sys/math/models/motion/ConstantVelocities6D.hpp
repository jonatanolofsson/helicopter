#pragma once
#ifndef SYS_MATH_MODELS_CONSTANT_VELOCITY_6D_HPP_
#define SYS_MATH_MODELS_CONSTANT_VELOCITY_6D_HPP_

#include <Eigen/Core>
#include <sys/types.hpp>
#include <sys/math/states.hpp>
#include <sys/math/constants.hpp>
#include <sys/math/algorithm.hpp>
#include <cmath>

#include <iostream>

namespace sys {
    namespace math {
        namespace models {
            using namespace Eigen;
            template<typename ExternalStateDescription_, typename S_ = Scalar>
            struct ConstantVelocities6D {
                typedef ConstantVelocities6D<ExternalStateDescription_, S_> Self;
                typedef ExternalStateDescription_ ExternalStateDescription;
                typedef S_ Scalar;
                typedef typename ExternalStateDescription::StateVector ExternalStateVector;
                typedef ExternalStateDescription extstates;
                typedef SCart3DQuat<Scalar, Self> States;
                typedef C0<Self> Controls;
                typedef typename States::StateVector Result;
                static const unsigned nofStates = States::nofStates;
                static const unsigned nofControls = Controls::nofControls;
                static const bool isDiscrete = true;

                static Result predict(const ExternalStateVector& x, const Scalar dT) {
                    Result xnext;
                    USING_XYZ

                    xnext.template segment<3>(States::position) = x.template segment<3>(extstates::position) + x.template segment<3>(extstates::velocity) * dT;
                    xnext.template segment<3>(States::velocity) = x.template segment<3>(extstates::velocity);
                    xnext.template segment<3>(States::omega) = x.template segment<3>(extstates::omega);

                    // Note that this skew-symmetric matrix is diagonally shifted upwards-left compared
                    // to many representations, due to the storage model used by Eigen for quaternions: qx, qy, qz, qw
                    Matrix<Scalar, 4, 4> S;
                    S << QUATERNION_ROTATION_FROM_ROTVEL(x(extstates::omegas[X]), x(extstates::omegas[Y]), x(extstates::omegas[Z]));

                    const Scalar wnorm     = x.template segment<3>(extstates::omega).norm();

                    if(wnorm > math::constants::EPSILON) {
                        const Scalar wnormT2   = wnorm * dT / 2;
                        xnext.template segment<4>(States::quaternion) =
                            (std::cos(wnormT2) * x.template segment<4>(extstates::quaternion)
                                - (std::sin(wnormT2) / wnorm) * S * x.template segment<4>(extstates::quaternion)
                            ).normalized();
                    } else {
                        xnext.template segment<4>(States::quaternion) = x.template segment<4>(extstates::quaternion);
                    }

                    return xnext;
                }

                static auto
                systemJacobian(const ExternalStateVector& x) -> decltype(math::template differentiate<States, Scalar>(ExternalStateVector(),Scalar()))
                {
                    return math::template differentiate<States>(x);
                }

                static Matrix<Scalar, ExternalStateDescription::nofStates, ExternalStateDescription::nofStates>
                covariance(const Scalar dT) {
                    typedef Matrix<Scalar, ExternalStateDescription::nofStates, ExternalStateDescription::nofStates> RetType;
                    return RetType::Identity() * dT;
                }
               
                static void update(ExternalStateVector& x, const Scalar dT) {
                    Result r = predict(x, dT);
                    for(int i=0; ++i; i < States::nofStates) {
                        x(States::statemap(i)) = r(i);
                    }
                }
    
            };
        }
    }
}

#endif
