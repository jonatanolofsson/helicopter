#pragma once
#ifndef SYS_MATH_MODELS_CONSTANT_VELOCITY_3D_HPP_
#define SYS_MATH_MODELS_CONSTANT_VELOCITY_3D_HPP_

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
            struct ConstantVelocities3D {
                typedef ModelDescription_ ModelDescription;
                typedef ConstantVelocities3D<ModelDescription> Self;
                typedef typename ModelDescription::Scalar Scalar;
                typedef typename ModelDescription::States States;
                static const bool isDiscrete = true;

                static States predict(const States& x, const Scalar dT) {
                    typedef typename ModelDescription::StateDescription states;
                    typedef typename ModelDescription::ControlDescription controls;
                    States xnext(x);

                    xnext.template segment<3>(states::position) += x.template segment<3>(states::velocity) * dT;
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
