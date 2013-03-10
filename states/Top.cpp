#include <sys/states/Top.hpp>
#include <os/com/messageDispatcher.hpp>

namespace sys {
    namespace states {
        Top::Top() : maple("/dev/ttyUSB0") {
            maple.registerPackager<MapleMessages::sensorMessage>(&os::messageDispatcher<SensorMessage>);
            maple.registerPackager<MapleMessages::controlMessage>(&os::messageDispatcher<ControlMessage>);
            maple.registerPackager<MapleMessages::cameraControlMessage>(&os::messageDispatcher<CameraControlMessage>);
            maple.registerPackager<MapleMessages::ioctlMessage>(&os::messageDispatcher<IoctlMessage>);
        }
    }
}
