#include <sys/math/models/models.hpp>
#include <sys/observer/observer.hpp>
#include <sys/math/filtering/KalmanFilter.hpp>
#include <sys/math/filtering/ExtendedKalmanFilter.hpp>
#include <sys/settings.hpp>
#include <os/clock.hpp>
#include <os/com/Executor.hpp>

namespace sys {
    using namespace math;

    typedef state_description::S6DOFQ StateDescription;
    typedef models::motion::ConstantVelocities  MotionModel;
    typedef ExtendedKalmanFilter<MotionModel> filter;

    typedef KalmanFilter<StateDescription> SystemState;
    SystemState system_state(sys::settings::dT);

    namespace observer {
        void timeUpdate(const os::SystemTime time) {
            filter::timeUpdate(system_state);
        }

        namespace {
            os::com::Executor<os::SystemTime> e(timeUpdate);

            struct InitializeObserver {
                InitializeObserver() {
                    StateDescription::initialize(system_state);
                }
            } init;
        }
    }
}
