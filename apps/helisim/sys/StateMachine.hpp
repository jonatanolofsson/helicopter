#pragma once
#ifndef APP_SYS_STATEMACHINE_HPP_
#define APP_SYS_STATEMACHINE_HPP_

#include <sys/statemachine/StateMachine.hpp>

namespace sys {
    namespace states {
        struct Top;
    }
    namespace statemachine {
        typedef states::Top InitialState;
        typedef StateEngine<InitialState> StateMachineEngine;
    }

    typedef statemachine::StateMachine<statemachine::InitialState> StateMachine;
}

#include <sys/states/Top.hpp>

#endif
