#ifndef SYS_MATH_CONTROL_CONTROLSTATE_HPP_
#define SYS_MATH_CONTROL_CONTROLSTATE_HPP_

#include <os/mem/ProtectedData.hpp>
#include <sys/types.hpp>
#include <sys/math/control.hpp>
#include <sys/math/filtering.hpp>

namespace sys {
    namespace math {
        template<typename M, typename S = Scalar>
        struct ControlState {
            typedef S Scalar;
            typedef typename StateVector<Scalar, M::nofStates>::type States;
            typedef typename ControlVector<Scalar, M::nofControls>::type Controls;
            typedef States Reference;
            typedef M Model;
            typedef ControlState<M,S> Self;
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
                : state(c.template segment<M::nofStates>(0))
            {}
        };
    }
}

#endif
