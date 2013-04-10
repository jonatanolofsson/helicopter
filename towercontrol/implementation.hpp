#pragma once
#ifndef SYS_ACTUATOR_IMPLEMENTATION_HPP_
#define SYS_ACTUATOR_IMPLEMENTATION_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/settings.hpp>
#include <sys/com/MotionControlSignal.hpp>
#include <sys/com/CameraControlSignal.hpp>
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
        , controlActuator(actuateControl)
        {}

        template<typename Serial>
        void TowerControl<Serial>::setTowerAngle(const U16 c) {
            currentAngle = c;
            stm.template send<>(stm::Messages::ById<Messages::towerMessage>::Type{currentAngle});
        }

        void actuateControl(const os::SystemTime systemTime) {
            U16 reference;
            {
                std::unique_lock<std::mutex> l(irGuard);
                if(firePosition[0] > 0) { // Light in site
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

        void irFirePosition(const stm::Messages::IrCameraMessage irPos) {
            std::cout << "Got light position: " << irPos.blobs[0][0] << ", " << irPos.blobs[0][1] << std::endl;
            std::unique_lock<std::mutex> l(irGuard);
            firePosition[0] = irPos.blobs[0][0];
            firePosition[1] = irPos.blobs[0][1];
        }

        void fireWater() {
            stm.template send<>(stm::Messages::ById<Messages::waterMessage>::Type{1});
        }
    }
}

#endif
