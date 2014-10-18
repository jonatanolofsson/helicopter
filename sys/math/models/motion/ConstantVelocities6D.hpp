#pragma once
#ifndef SYS_MATH_MODELS_CONSTANT_VELOCITY_6D_HPP_
#define SYS_MATH_MODELS_CONSTANT_VELOCITY_6D_HPP_

#include <Eigen/Core>
#include <sys/math/models/motion.hpp>
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
            struct ConstantVelocities6D {
                typedef ConstantVelocities6D Self;
                typedef SCart3DQuat States;
                typedef typename States::StateVector Result;
                static const int nofStates = States::nofStates;
                static const bool isDiscrete = true;


                template<typename ExternalStates>
                static Result predict(const typename ExternalStates::StateVector& x, const Scalar dT) {
                    typedef ExternalStates extstates;
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

                static Matrix<Scalar, nofStates, nofStates>
                covariance(const Scalar dT) {
                    typedef Matrix<Scalar, nofStates, nofStates> RetType;
                    return RetType::Identity() * dT;
                }

                template<typename ExternalStates>
                static void update(typename ExternalStates::StateVector& x, const Scalar dT) {
                    Result r = predict<ExternalStates>(x, dT);
                    for(int i=0; i < nofStates; ++i) {
                        x(States::template statemap<ExternalStates>(i)) = r(i);
                    }
                }
            };
        }
    }
}

#endif
