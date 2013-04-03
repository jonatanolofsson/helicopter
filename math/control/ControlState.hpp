#pragma once
#ifndef SYS_MATH_CONTROL_CONTROLSTATE_HPP_
#define SYS_MATH_CONTROL_CONTROLSTATE_HPP_

#include <os/mem/ProtectedData.hpp>
#include <sys/types.hpp>
#include <sys/math/control.hpp>
#include <sys/math/filtering.hpp>

namespace sys {
    namespace math {
        template<typename ModelDescription>
        struct ControlState {
            typedef ControlState<ModelDescription> Self;
            typedef typename ModelDescription::Scalar Scalar;
            typedef typename ModelDescription::States States;
            typedef typename ModelDescription::Controls Controls;
            typedef States Reference;

            States state;
            Controls controls;

            explicit ControlState() {
                state.setZero();
            }

            ControlState(const Self& c)
                : state(c.state)
                , controls(c.controls)
            {}

            template<typename T>
            explicit ControlState(const T& c)
                : state(c.template segment<ModelDescription::nofStates>(0))
            {}
        };
    }
}

#endif
