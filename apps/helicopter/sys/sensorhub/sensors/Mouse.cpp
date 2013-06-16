#include <sys/Sensorhub.hpp>
#include <sys/math/models.hpp>
#include <sys/math/filtering.hpp>
#include <os/com/getSignal.hpp>
#include <os/com/com.hpp>
#include <linux/input.h>
#include <os/clock.hpp>
#include <sys/settings.hpp>
#include <mutex>

#define MOUSEFILE "/dev/input/by-id/usb-Logitech_Gaming_Mouse_G400-event-mouse"

INSTANTIATE_SIGNAL(sys::sensorhub::sensors::Mouse);

namespace sys {
    namespace sensorhub {
        Mouse::Mouse()
        : dispatcher(&Mouse::timelyReader, this)
        , readerThread(&Mouse::mouseReader, this)
        {}

        void Mouse::timelyReader(const os::SystemTime) {
            typedef sys::sensorhub::sensors::Mouse Measurement;
            Measurement m;
            std::unique_lock<std::mutex> l(inputGuard);

            m.z[0] = dots * (settings::systemFrequency * 2.54) / 40000.0;
            m.R << 0.1; // FIXME

            static Scalar distance = 0;
            distance += m.z[0] * settings::dT;

            //~ std::cout << "Distance, Velocity: " << distance << " :: " << m.z[0] << std::endl;

            dots = 0;
            os::yield(m);
        }

        void Mouse::mouseReader() {
            struct input_event ie;
            unsigned int readData;

            int fd = open(MOUSEFILE, O_RDONLY);
            if(fd > 0) {
                while(!dying) {
                    readData = read(fd, &ie, sizeof(struct input_event));
                    if(readData == sizeof(struct input_event)) {
                        if((ie.type == EV_REL) && (ie.code==0)) {
                            std::unique_lock<std::mutex> l(inputGuard);
                            dots += ie.value;
                        }
                    }
                }
                close(fd);
            }
            std::cout << "Mouse thread died" << std::endl;
        }

        Mouse::~Mouse() {
            dying = true;
        }
    }
}
