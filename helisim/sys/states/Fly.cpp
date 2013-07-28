#include <sys/ReferenceGenerator.hpp>
#include <sys/states/Fly.hpp>

namespace sys {
    namespace states {
        ReferenceGenerator::TemporalReference ref[] = {
            {0, (ReferenceGenerator::Reference() << 0,0,0).finished()},
            {10, (ReferenceGenerator::Reference() << 1,2,0).finished()}
        };
        Fly::Fly() 
        : referenceGenerator(ref, sizeof(ref)/sizeof(ref[0]))
        {
            os::startTime();
        }
    }
}
