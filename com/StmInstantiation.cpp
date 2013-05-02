#include <os/com/getSignal.hpp>
#include <sys/com/Stm.hpp>

INSTANTIATE_SIGNAL(sys::stm::SensorMessage);
INSTANTIATE_SIGNAL(sys::stm::ControlMessage);
INSTANTIATE_SIGNAL(sys::stm::IrCameraMessage);
INSTANTIATE_SIGNAL(sys::stm::TowerMessage);
INSTANTIATE_SIGNAL(sys::stm::IoctlMessage);
