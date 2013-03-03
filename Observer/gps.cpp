#include <sys/Observer/Observer.hpp>
#include <os/com/Dispatcher.hpp>
#include <sys/Observer/gps.hpp>

namespace sys {
    namespace Observer {
        os::Dispatcher<Measurement<GPS<>>> e(measurementUpdate<GPS<Scalar>>);
        //~ i2c<0>::registerPackager(TestMessages::MY_MSG, gpsPackager);
    }
}
