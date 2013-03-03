#include <sys/states/Top.hpp>
#include <os/com/messageDispatcher.hpp>

namespace sys {
    Top::Top() : maple("/dev/ttyUSB0") {
        maple.registerPackager<MapleMessages::sensorMessage>(&os::messageDispatcher<SensorMessage>);
        maple.registerPackager<MapleMessages::controlMessage>(&os::messageDispatcher<ControlMessage>);
        maple.registerPackager<MapleMessages::cameraControlMessage>(&os::messageDispatcher<CameraControlMessage>);
        maple.registerPackager<MapleMessages::ioctlMessage>(&os::messageDispatcher<IoctlMessage>);

        imageProcessor.registerPackager<ImageProcessorMessages::cameraMessage>(&os::messageDispatcher<CameraMessage>);
        imageProcessor.registerPackager<ImageProcessorMessages::gpsMessage>(&os::messageDispatcher<GpsMessage>);
        imageProcessor.registerPackager<ImageProcessorMessages::ipCtlMessage>(&os::messageDispatcher<IpCtlMessage>);
    }
}
