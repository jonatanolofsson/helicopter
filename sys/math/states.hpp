#pragma once
#ifndef SYS_MATH_STATES_STATES_HPP_
#define SYS_MATH_STATES_STATES_HPP_
#include <sys/math/filtering.hpp>
#include <sys/com/statemessage.hpp>

namespace sys {
    namespace math {
        namespace models {
            template<typename Self_, int NOF_STATES>
            struct State {
                typedef Self_ Self;
                static const int nofStates = NOF_STATES;
                typedef math::internal::StateVector<nofStates> StateVector;
                typedef math::internal::Covariance<nofStates> Covariance;
                typedef messages::StateMessage<StateVector> StateMessage;

                template<typename ExternalStates>
                StateVector extract(const typename ExternalStates::StateVector& x) {
                    StateVector r;
                    for(int i = 0; i < nofStates; ++i) {
                        r(i) = x(Self::template statemap<ExternalStates>(i));
                    }
                    return r;
                }

                template<typename ExternalStates>
                static typename ExternalStates::StateVector translate(const StateVector& x) {
                    typename ExternalStates::StateVector res; res.setZero();
                    for(int i = 0; i < nofStates; ++i) {
                        res(Self::template statemap<ExternalStates>(i)) = x(i);
                    }
                    return res;
                }

                template<typename ExternalStates>
                static typename ExternalStates::Covariance translate(const Covariance& x) {
                    typename ExternalStates::Covariance res; res.setZero();
                    for(int i = 0; i < nofStates; ++i) {
                        for(int j = 0; j < nofStates; ++i) {
                            res(Self::template statemap<ExternalStates>(i), Self::template statemap<ExternalStates>(j)) = x(i,j);
                        }
                    }
                    return res;
                }

                template<typename T>
                static void initializeState(T& state) {
                    state.setZero();
                }

                template<typename T>
                static void initialize(T& filter) {
                    auto l = filter.retrieve_lock();
                    initializeState(filter.state);
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
#include <sys/math/states/V_3D.hpp>
#include <sys/math/states/XQ_3D.hpp>
#include <sys/math/states/XYTVWA_1D.hpp>
#include <sys/math/states/XYT_1D.hpp>
#include <sys/math/states/X_3D.hpp>

#endif
