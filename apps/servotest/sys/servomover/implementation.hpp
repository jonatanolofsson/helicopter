#pragma once
#ifndef SYS_OBSERVER_IMPLEMENTATION_HPP_
#define SYS_OBSERVER_IMPLEMENTATION_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/Servomover.hpp>

#include <iostream>
#include <cmath>
#include <sys/settings.hpp>

namespace sys {
    namespace servomover {
        template<typename Trigger, typename Serial>
        Servomover<Trigger, Serial>::Servomover(Serial& maple_)
        : maple(maple_)
        , dispatcher(&Self::servoUpdate, this)
        , ctrlMsg()
        {}

        template<typename Trigger, typename Serial>
        void Servomover<Trigger, Serial>::servoUpdate(const Trigger t) {
            ctrlMsg.servo[0] = 512 * (1 + std::sin(t.value * settings::dT));
            if(t.value > 500) ctrlMsg.servo[1] = 512 * (1 + std::sin(t.value * settings::dT));
            if(t.value > 1000) ctrlMsg.servo[2] = 512 * (1 + std::sin(t.value * settings::dT));
            if(t.value > 1500) ctrlMsg.rpm = 512 * (1 + std::sin(t.value * settings::dT));
            maple.send(ctrlMsg);
        }
    }
}

#endif
