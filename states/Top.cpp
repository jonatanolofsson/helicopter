#include <sys/states/Top.hpp>
#include <os/com/messageDispatcher.hpp>
#include <sys/com/Maple.hpp>

namespace sys {
    namespace states {
        Top::Top() : maple("/dev/ttyUSB0") {
            maple.registerPackager<MapleMessages::sensorMessage>(&os::messageDispatcher<SensorMessage>);
            maple.registerPackager<MapleMessages::controlMessage>(&os::messageDispatcher<ControlMessage>);
            maple.registerPackager<MapleMessages::cameraControlMessage>(&os::messageDispatcher<CameraControlMessage>);
            maple.registerPackager<MapleMessages::ioctlMessage>(&os::messageDispatcher<IoctlMessage>);

            IoctlMessage ioctlMsg = { IoctlMessage::SEND_SENSOR_DATA };
            maple.send<>(ioctlMsg);
        }
        Top::~Top() {
            IoctlMessage ioctlMsg = { 0 };
            maple.send<>(ioctlMsg);
        }
    }
}
