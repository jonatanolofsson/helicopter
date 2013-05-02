#pragma once
#ifndef SYS_ACTUATOR_IMPLEMENTATION_HPP_
#define SYS_ACTUATOR_IMPLEMENTATION_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/settings.hpp>
#include <sys/com/Stm.hpp>
#include <stdlib.h>
#include <sys/actuator/API.hpp>
#include <sys/settings.hpp>

namespace sys {
    namespace towercontrol {
        template<typename Serial>
        TowerControl<Serial>::TowerControl(Serial& stm_)
        : currentAngle(0)
        , stm(stm_)
        , controlActuator(&TowerControl<Serial>::actuateControl, this)
        {}

        template<typename Serial>
        void TowerControl<Serial>::setTowerAngle(const U16 c) {
            currentAngle = c;
            stm.template send<>(stm::Messages::ById<stm::Messages::towerMessage>::Type{(S16)currentAngle});
        }

        template<typename Serial>
        void TowerControl<Serial>::actuateControl(const os::SystemTime systemTime) {
            U16 reference;
            {
                std::unique_lock<std::mutex> l(irGuard);
                if(firePosition[0] > 0) { // Light in sight
                    reference = currentAngle + regP * (512 - firePosition[0]);
                    if(std::abs(512 - firePosition[0]) < firePermissionOffset) {
                        fireWater();
                    }
                } else {
                    reference = maxAngle * std::sin(systemTime.value * settings::dT);
                }
            }
            setTowerAngle(reference);
        }

        template<typename Serial>
        void TowerControl<Serial>::irFirePosition(const stm::IrCameraMessage irPos) {
            std::cout << "Got light position: " << irPos.blobs[0][0] << ", " << irPos.blobs[0][1] << ", " << irPos.blobs[0][2] << std::endl;
            std::unique_lock<std::mutex> l(irGuard);
            firePosition[0] = irPos.blobs[0][0];
            firePosition[1] = irPos.blobs[0][1];
        }

        template<typename Serial>
        void TowerControl<Serial>::fireWater() {
            stm.template send<>(stm::Messages::ById<stm::Messages::waterMessage>::Type{1});
        }
    }
}

#endif
