#include <os/com/getSignal.hpp>
#include <sys/com/Stm.hpp>

INSTANTIATE_SIGNAL(sys::stm::IrSensorMessage);
INSTANTIATE_SIGNAL(sys::stm::ControlMessage);
INSTANTIATE_SIGNAL(sys::stm::IoctlMessage);
