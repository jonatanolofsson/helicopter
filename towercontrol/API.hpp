#pragma once
#ifndef SYS_TOWERCONTROL_API_HPP_
#define SYS_TOWERCONTROL_API_HPP_

#include <sys/towercontrol/TowerControl.hpp>
#include <sys/com/Stm.hpp>
#include <sys/types.hpp>

namespace sys {
    namespace towercontrol {
        typedef Stm SerialLink;
        static const U16 firePermissionOffset   = 50;
        static const Scalar regP                = 10.0;
        static const Scalar maxAngle            = 100.0;
    }
    typedef towercontrol::TowerControl<towercontrol::SerialLink> TowerControl;
}

#endif
