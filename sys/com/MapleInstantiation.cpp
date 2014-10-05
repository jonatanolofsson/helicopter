#include <os/com/getSignal.hpp>
#include <sys/com/Maple.hpp>

INSTANTIATE_SIGNAL(sys::maple::SensorMessage);
INSTANTIATE_SIGNAL(sys::maple::ControlMessage);
INSTANTIATE_SIGNAL(sys::maple::CameraControlMessage);
INSTANTIATE_SIGNAL(sys::maple::IoctlMessage);
