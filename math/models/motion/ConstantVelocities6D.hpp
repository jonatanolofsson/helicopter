#pragma once
#ifndef SYS_MATH_MODELS_CONSTANT_VELOCITY_6D_HPP_
#define SYS_MATH_MODELS_CONSTANT_VELOCITY_6D_HPP_

#include <Eigen/Core>
#include <sys/types.hpp>
#include <sys/math/constants.hpp>
#include <sys/math/algorithm.hpp>
#include <cmath>

#include <iostream>

namespace sys {
    namespace math {
        namespace models {
            using namespace Eigen;
            template<typename ModelDescription_>
            struct ConstantVelocities6D {
                typedef ModelDescription_ ModelDescription;
                typedef ConstantVelocities6D<ModelDescription> Self;
                typedef typename ModelDescription::Scalar Scalar;
                typedef typename ModelDescription::States States;
                static const bool isDiscrete = true;

                static States predict(const States& x, const Scalar dT) {
                    typedef typename ModelDescription::StateDescription states;
                    typedef typename ModelDescription::ControlDescription controls;
                    States xnext(x);
                    USING_XYZ

                    xnext.template segment<3>(states::position) += x.template segment<3>(states::velocity) * dT;

                    // Note that this skew-symmetric matrix is diagonally shifted upwards-left compared
                    // to many representations, due to the storage model used by Eigen for quaternions: qx, qy, qz, qw
                    Matrix<Scalar, 4, 4> S;
                    S << QUATERNION_ROTATION_FROM_ROTVEL(x(states::omega[X]), x(states::omega[Y]), x(states::omega[Z]));

                    const Scalar wnorm     = x.template segment<3>(states::rotational_velocity).norm();
                    const Scalar wnormT2   = wnorm * dT / 2;

                    if(wnorm > math::constants::EPSILON) {
                        xnext.template segment<4>(states::quaternion) =
                            (std::cos(wnormT2) * x.template segment<4>(states::quaternion)
                                - (std::sin(wnormT2) / wnorm) * S * x.template segment<4>(states::quaternion)
                            ).normalized();
                    }

                    return xnext;
                }

                template<typename D>
                static States diffStates(const States& x, const D& dx) {
                    return predict(x + dx, 1e-2) - predict(x - dx, 1e-2);
                }

                static Matrix<Scalar, ModelDescription::nofStates, ModelDescription::nofStates>
                systemJacobian(const States& x) {
                    static const States dx = States::Constant(1e-2);
                    return math::template differentiateStates<ModelDescription, &Self::diffStates>(x, dx);
                }

                static Matrix<Scalar, ModelDescription::nofStates, ModelDescription::nofStates>
                covariance(const Scalar dT) {
                    typedef Matrix<Scalar, ModelDescription::nofStates, ModelDescription::nofStates> RetType;
                    return RetType::Identity() * dT;
                }
            };
        }
    }
}

#endif
