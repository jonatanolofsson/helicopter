#ifndef SYS_MODELS_NOCONTROL_HPP_
#define SYS_MODELS_NOCONTROL_HPP_

namespace sys {
    namespace math {
        namespace models {
            struct NoControl {
                template<typename T>
                static void initialize(T& filter) {}

                static const int nofControls = 0;
            };
        }
    }
}

#endif
