#include <sys/ReferenceGenerator.hpp>
#include <sys/states/Fly.hpp>
#include <sys/math/base.hpp>

namespace sys {
    namespace states {
        static_assert(ReferenceGenerator::Reference::RowsAtCompileTime == 6, "Wrong number of elements in reference");
        ReferenceGenerator::TemporalReference ref[] = {
            {0,    (ReferenceGenerator::Reference() << 0,0,0,0,0,0).finished()},
            {10,   (ReferenceGenerator::Reference() << 0,0,0,1,2,0).finished()},
            {20,   (ReferenceGenerator::Reference() << 0,0,0,0,0,0).finished()}
        };
        Fly::Fly() 
        : referenceGenerator(ref, countof(ref))
        {
            os::startTime();
        }
    }
}
