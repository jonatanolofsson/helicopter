#pragma once
#ifndef SYS_MATH_MODELS_HELICOPTERCONTROL_HPP_
#define SYS_MATH_MODELS_HELICOPTERCONTROL_HPP_

#include <Eigen/Core>
#include <sys/types.hpp>
#include <sys/math/constants.hpp>
#include <sys/math/algorithm.hpp>
#include <sys/math/models/motion/Helicopter.hpp>
#include <cmath>

#include <iostream>

namespace sys {
    namespace math {
        namespace models {
            using namespace Eigen;
            template<typename ModelDescription_, typename IModelDescription_ = ModelDescription_>
            struct HelicopterControl {
                typedef Helicopter<ModelDescription_> Self;
                typedef IModelDescription_ IModelDescription;
                typedef ModelDescription_ ModelDescription;
                typedef typename ModelDescription::Scalar Scalar;
                typedef typename ModelDescription::States States;
                typedef typename ModelDescription::Controls Controls;
                typedef typename ModelDescription::ControlDescription controls;
                static const bool isDiscrete = false;

                template<typename IModelDescription = IModelDescription>
                static States derivative(const typename IModelDescription::States& x, const typename IModelDescription::Controls& u) {
                    return Helicopter<ModelDescription>::template accPredition<IModelDescription>(x, u);
                    /*
                     *return (States() << 
                     *        Helicopter<ModelDescription>::template accPredition<IModelDescription>(x, u), 
                     *        Helicopter<ModelDescription>::template controlDerivative<IModelDescription>(x, u)).finished();
                     */
                }

                template<typename D>
                static States diffStates(const typename IModelDescription::States& x, const typename IModelDescription::Controls& u, const D& dx) {
                    auto xminus = x; xminus.template segment<ModelDescription::nofStates>(0) -= dx;
                    auto xplus  = x; xplus.template segment<ModelDescription::nofStates>(0) += dx;
                    return derivative(xplus, u) - derivative(xminus, u);
                }

                template<typename D>
                static States diffControls(const typename IModelDescription::States& x, const typename IModelDescription::Controls& u, const D& du) {
                    auto uminus = u; uminus.template segment<ModelDescription::nofControls>(0) -= du;
                    auto uplus  = u; uplus.template segment<ModelDescription::nofControls>(0) += du;
                    return derivative(x, uplus) - derivative(x, uminus);
                }

                template<typename IModelDescription = IModelDescription>
                static Matrix<Scalar, ModelDescription::nofStates, ModelDescription::nofStates>
                systemJacobian(const typename IModelDescription::States& x, const typename IModelDescription::Controls& u) {
                    static const States dx = States::Constant(1e-2);
                    return math::template differentiateStates<ModelDescription, typename IModelDescription::States, typename IModelDescription::Controls, States, &HelicopterControl<ModelDescription, IModelDescription>::diffStates>(x, u, dx);
                }

                static Matrix<Scalar, ModelDescription::nofStates, ModelDescription::nofStates>
                covariance(const Scalar dT) {
                    typedef Matrix<Scalar, ModelDescription::nofStates, ModelDescription::nofStates> RetType;
                    return RetType::Identity() * dT;
                }

                template<typename IModelDescription = IModelDescription>
                static Matrix<Scalar, ModelDescription::nofStates, ModelDescription::nofControls>
                controlJacobian(const typename IModelDescription::States& x, const typename IModelDescription::Controls& u) {
                    static const Controls du = Controls::Constant(1e-2);
                    return math::template differentiateStates<ModelDescription, typename IModelDescription::States, typename IModelDescription::Controls, Controls, &HelicopterControl<ModelDescription, IModelDescription>::diffControls>(x, u, du);
                }
            };
        }
    }
}

#endif
