#include <sys/Observer/Observer.hpp>
#include <os/com/Dispatcher.hpp>
#include <sys/Observer/gps.hpp>

namespace sys {
    namespace observer {
        os::Dispatcher<GaussianMeasurement<GPS<>>> e(measurementUpdate<GPS<Scalar>>);
    }
}
