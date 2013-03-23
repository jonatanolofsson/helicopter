#pragma once
#ifndef SYS_STATES_IMUTEST1_HPP_
#define SYS_STATES_IMUTEST1_HPP_
#include <sys/states/Init.hpp>
#include <iostream>

namespace sys {
    namespace states {
        struct ImuTest2;
        struct ImuTest1
        : sc::state<ImuTest1, Init>
        {
            typedef mpl::list<
                sc::transition< events::FlippedDown, ImuTest2 >
            > reactions;

            ImuTest1(my_context ctx) : my_base(ctx) {
                context<Top>().sensorhub.imu.up = true;
                std::cout << "ImuTest1" << std::endl;
            }
        };
    }
}

#endif
