#pragma once
#ifndef SYS_TOWERCONTROL_HPP_
#define SYS_TOWERCONTROL_HPP_

#include <os/com/Dispatcher.hpp>
#include <os/clock.hpp>
#include <sys/types.hpp>
#include <mutex>
#include <sys/com/Stm.hpp>

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
                void irFirePosition(const stm::IrCameraMessage irPos);
                std::mutex irGuard;

                Serial& stm;

                os::Dispatcher<TowerControl, os::SystemTime> controlActuator;

            public:
                explicit TowerControl(Serial& stm_);
        };
    }
}

#endif
