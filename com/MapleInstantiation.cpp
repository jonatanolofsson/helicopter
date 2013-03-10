#include <os/com/getSignal.hpp>
#include <sys/com/Maple.hpp>

INSTANTIATE_SIGNAL(sys::SensorMessage);
INSTANTIATE_SIGNAL(sys::ControlMessage);
INSTANTIATE_SIGNAL(sys::CameraControlMessage);
INSTANTIATE_SIGNAL(sys::IoctlMessage);
