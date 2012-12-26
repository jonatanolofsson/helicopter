#ifndef SYS_MOTION_CONTROL_HPP_
#define SYS_MOTION_CONTROL_HPP_

#include <sys/settings.hpp>
#include <os/com/Dispatcher.hpp>
#include <Eigen/Core>
#include <sys/types.hpp>

namespace sys {
    class MotionControl {
        private:
            MainController mainController;
            os::Dispatcher<MotionControl, SystemModel, SystemState> dispatcher;

            void updateControl(const SystemModel m, const SystemState x) {
                mainController.updateModel(m);
                yield(mainController(x));
            }

        public:
            MotionControl()
            : dispatcher(updateControl)
            {}
    }
}

#endif
