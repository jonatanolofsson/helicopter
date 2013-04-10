#pragma once
#ifndef SYS_ACTUATOR_HPP_
#define SYS_ACTUATOR_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/types.hpp>

namespace sys {
    namespace towercontrol {
        template<typename Serial>
        class TowerControl {
            private:
                U16 firePosition[2];
                U16 currentAngle;
                void actuateControl(const os::SystemTime);
                void setTowerAngle(const U16 c);
                void fireWater();

                Serial& stm;

                os::Dispatcher<TowerControl, os::SystemTime> controlActuator;

                TowerControl();

            public:
                explicit TowerControl(Serial& stm_);
        };
    }
}

#endif
