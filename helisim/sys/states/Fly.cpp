#include <sys/ReferenceGenerator.hpp>
#include <sys/states/Fly.hpp>

namespace sys {
    namespace states {
        ReferenceGenerator::TemporalReference ref[] = {
            {0, (ReferenceGenerator::Reference() << 0,0,0).finished()}
        };
        Fly::Fly() 
        : referenceGenerator(ref, 1)
        {
            os::startTime();
        }
    }
}
