#pragma once
#ifndef SYS_MODELS_NONE_HPP_
#define SYS_MODELS_NONE_HPP_

namespace sys {
    namespace math {
        namespace models {
            template<typename Model_>
            struct C0 {
                typedef Model_ Model;
                static const int nofStates = 0;
            };
        }
    }
}

#endif
