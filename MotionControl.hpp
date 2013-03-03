#ifndef SYS_MOTION_CONTROL_HPP_
#define SYS_MOTION_CONTROL_HPP_

#include <sys/settings.hpp>
#include <os/com/Dispatcher.hpp>
#include <sys/types.hpp>
#include <sys/math/control/LqController.hpp>

#include <sys/math/filtering/KalmanFilter.hpp>
#include <sys/math/models/DirectVelocities.hpp>
#include <sys/math/models/S6CARTQ.hpp>
#include <sys/math/models/C6VW.hpp>

namespace sys {
    typedef math::S6CARTQ StateDescription;
    typedef math::C6VW ControlDescription;
    typedef StateDescription states;
    typedef math::KalmanFilter<StateDescription, ControlDescription> SystemState;
    typedef models::motion::DirectVelocities MotionModel;

    typedef math::LqController<Scalar, StateDescription::number_of_states, ControlDescription::number_of_controls, MotionModel::CD> MainController;

    class MotionControl {
        private:
            MainController mainController;
            os::Dispatcher<MotionControl, SystemState> dispatcher;

            template<typename M>
            void updateControl(const SystemState x) {
                mainController.updateModel(M::template systemJacobian<SystemState>(x), M::template controlJacobian<SystemState>(x));
                os::yield(mainController(x.state));
            }

        public:
            MotionControl()
            : dispatcher(&MotionControl::updateControl<MotionModel>, this)
            {}
    };
}

#endif
