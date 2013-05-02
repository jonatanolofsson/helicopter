#pragma once
#ifndef SYS_STATES_IMUTEST2_HPP_
#define SYS_STATES_IMUTEST2_HPP_
#include <sys/states/Init.hpp>
#include <iostream>

namespace sys {
    namespace states {
        struct ImuTest1;
        struct ImuTest2
        : sc::state<ImuTest2, Init>
        {
            typedef mpl::list<
                sc::transition< events::FlippedUp, ImuTest1 >
            > reactions;

            ImuTest2(my_context ctx) : my_base(ctx) {
                context<Top>().sensorhub.imu.up = false;
                std::cout << "ImuTest2" << std::endl;
                //~ context<Top>().debugServer.sendString("Ner\n");
            }
        };
    }
}

#endif
