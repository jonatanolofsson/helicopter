#pragma once
#ifndef SYS_MATH_STATES_STATES_HPP_
#define SYS_MATH_STATES_STATES_HPP_
#include <Eigen/Core>
#include <sys/math/filtering.hpp>
#include <sys/com/statemessage.hpp>

namespace sys {
    namespace math {
        namespace models {
            using namespace Eigen;
            template<typename Parent_, int NOF_STATES>
            struct State {
                typedef Parent_ Parent;
                static const int nofStates = NOF_STATES;
                typedef math::internal::StateVector<nofStates> StateVector;
                typedef math::internal::Covariance<nofStates> Covariance;
                typedef StateMessage<StateVector> StateMessage;

                template<typename ExternalStates>
                static StateVector translate(const typename ExternalStates::StateVector& x) {
                    StateVector res; res.setZero();
                    for(int i = 0; i < nofStates; ++i) {
                        res(i) = x(Parent::template statemap<ExternalStates>(i));
                    }
                    return res;
                }

                template<typename ExternalStates>
                static Covariance translate(const typename ExternalStates::Covariance& x) {
                    Covariance res; res.setZero();
                    for(int i = 0; i < nofStates; ++i) {
                        for(int j = 0; j < nofStates; ++i) {
                            res(i,j) = x(Parent::template statemap<ExternalStates>(i),
                                         Parent::template statemap<ExternalStates>(j));
                        }
                    }
                    return res;
                }

                template<typename ExternalStates>
                static void update(typename ExternalStates::StateVector& x, const StateVector& xmine) {
                    for(int i = 0; i < nofStates; ++i) {
                        x(Parent::template statemap<ExternalStates>(i)) = xmine(i);
                    }
                }

                template<typename T>
                static void initializeState(T& state) {
                    state.setZero();
                }

                template<typename T>
                static void initialize(T& filter) {
                    auto l = filter.retrieve_lock();
                    Parent::initializeState(filter.state);
                    filter.covariance.setIdentity();
                }
            };
        }
    }
}

#include <sys/math/states/Helicopter.hpp>
#include <sys/math/states/HelicopterControls.hpp>
#include <sys/math/states/VWXALQ_3D.hpp>
#include <sys/math/states/VWXQ_3D.hpp>
#include <sys/math/states/VW_1D.hpp>
#include <sys/math/states/VW_3D.hpp>
#include <sys/math/states/AW_3D.hpp>
#include <sys/math/states/V_3D.hpp>
#include <sys/math/states/XQ_3D.hpp>
#include <sys/math/states/XYTVWA_1D.hpp>
#include <sys/math/states/XYT_1D.hpp>
#include <sys/math/states/X_3D.hpp>

#endif
