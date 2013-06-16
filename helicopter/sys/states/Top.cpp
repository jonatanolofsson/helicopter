#include <sys/states/Top.hpp>
#include <os/com/messageDispatcher.hpp>
#include <sys/com/Maple.hpp>
#include <iostream>

namespace sys {
    namespace states {
        Top::Top()
        : maple("/dev/maple")
        {
            maple.registerPackager<maple::Messages::sensorMessage>(&os::messageDispatcher<maple::SensorMessage>);
            maple.registerPackager<maple::Messages::controlMessage>(&os::messageDispatcher<maple::ControlMessage>);
            maple.registerPackager<maple::Messages::cameraControlMessage>(&os::messageDispatcher<maple::CameraControlMessage>);
            maple.registerPackager<maple::Messages::ioctlMessage>(&os::messageDispatcher<maple::IoctlMessage>);

            {
                maple::IoctlMessage ioctlMsg = { maple::IoctlMessage::SEND_SENSOR_DATA };
                maple.send<>(ioctlMsg);
            }

            /*
             * while(true) {
             *     if(debugServer.connected()) {
             *         debugServer.sendString("Teststring\n");
             *     }
             * }
             */
        }


        Top::~Top() {
            maple::IoctlMessage ioctlMsg = { 0 };
            maple.send<>(ioctlMsg);
        }
    }
}
