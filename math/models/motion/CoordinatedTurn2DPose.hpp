#pragma once
#ifndef SYS_MATH_MODELS_COORDINATEDTURN2DPOSE_HPP_
#define SYS_MATH_MODELS_COORDINATEDTURN2DPOSE_HPP_

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
            struct CoordinatedTurn2DPose {
                typedef ModelDescription_ ModelDescription;
                typedef CoordinatedTurn2DPose<ModelDescription> Self;
                typedef typename ModelDescription::Scalar Scalar;
                typedef typename ModelDescription::States States;
                typedef typename ModelDescription::Controls Controls;
                static const bool isDiscrete = true;

                static States predict(const States& x, const Controls& u, const Scalar dT) {
                    typedef typename ModelDescription::StateDescription states;
                    typedef typename ModelDescription::ControlDescription controls;
                    States xnext(x);
                    using std::sin; using std::cos;

                    Scalar a;
                    if(u(controls::w) < 1e-2) {
                        a = u(controls::v) * dT;
                    } else {
                        a = 2. * u(controls::v) * sin(u(controls::w) * dT / 2.) / u(controls::w);
                    }
                    const auto b = x(states::th) + (u(controls::w) * dT / 2.);

                    xnext(states::x) += a*cos(b);
                    xnext(states::y) += a*sin(b);
                    xnext(states::th) += u(controls::w) * dT;
                    return xnext;
                }

                template<typename D>
                static States diffStates(const States& x, const Controls& u, const D& dx) {
                    return predict(x + dx, u, 1e-2) - predict(x - dx, u, 1e-2);
                }

                template<typename D>
                static States diffControls(const States& x, const Controls& u, const D& du) {
                    return predict(x, u + du, 1e-2) - predict(x, u - du, 1e-2);
                }

                static Matrix<Scalar, ModelDescription::nofStates, ModelDescription::nofStates>
                systemJacobian(const States& x, const Controls& u) {
                    static const States dx = States::Constant(1e-2);
                    return math::template differentiateStates<ModelDescription, States, &Self::diffStates>(x, u, dx);
                }

                static Matrix<Scalar, ModelDescription::nofStates, ModelDescription::nofControls>
                controlJacobian(const States& x, const Controls& u) {
                    static const Controls du = Controls::Constant(1e-2);
                    return math::template differentiateStates<ModelDescription, Controls, &Self::diffControls>(x, u, du);
                }

                static Matrix<Scalar, ModelDescription::nofStates, ModelDescription::nofStates>
                covariance(const Scalar dT) {
                    typedef Matrix<Scalar, ModelDescription::nofStates, ModelDescription::nofStates> RetType;
                    return RetType::Identity() * dT * 0.01;
                }
            };
        }
    }
}

#endif
