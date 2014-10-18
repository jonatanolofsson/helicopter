#pragma once
#ifndef SYS_MATH_MODELS_COORDINATEDTURN2D_HPP_
#define SYS_MATH_MODELS_COORDINATEDTURN2D_HPP_

#include <Eigen/Core>
#include <sys/types.hpp>
#include <sys/math/constants.hpp>
#include <sys/math/algorithm.hpp>
#include <cmath>

namespace sys {
    namespace math {
        namespace models {
            using namespace Eigen;
            template<typename ModelDescription_>
            struct CoordinatedTurn2D {
                typedef ModelDescription_ ModelDescription;
                typedef CoordinatedTurn2D<ModelDescription> Self;
                typedef typename ModelDescription::Scalar Scalar;
                typedef typename ModelDescription::States States;
                typedef typename ModelDescription::Controls Controls;
                static const bool isDiscrete = true;

                static States predict(const States& x, const Scalar dT) {
                    typedef typename ModelDescription::StateDescription states;
                    States xnext(x);
                    using std::sin; using std::cos;

                    const auto a = 2. * x(states::v) * sin(x(states::w) * dT / 2.) / x(states::w);
                    const auto b = x(states::th) + (x(states::w) * dT / 2.);

                    xnext(states::x) += a*cos(b);
                    xnext(states::y) += a*sin(b);
                    xnext(states::th) += x(states::w)*dT;
                    xnext(states::v) += x(states::a)*dT;
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
