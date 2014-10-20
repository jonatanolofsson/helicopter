#include <sys/ReferenceGenerator.hpp>
#include <sys/states/Fly.hpp>
#include <sys/math/base.hpp>

namespace sys {
    namespace states {
        /*
         *static_assert(ReferenceGenerator::ReferenceVector::RowsAtCompileTime == 6, "Wrong number of elements in reference");
         *ReferenceGenerator::TemporalReference ref[] = {
         *    {0,    (ReferenceGenerator::ReferenceVector() << 0,0,0,0,0,0).finished()},
         *    {10,   (ReferenceGenerator::ReferenceVector() << 0,0,0,1,2,0).finished()},
         *    {20,   (ReferenceGenerator::ReferenceVector() << 0,0,0,0,0,0).finished()}
         *};
         */
        static_assert(ReferenceGenerator::ReferenceVector::RowsAtCompileTime == 7, "Wrong number of elements in reference");
        ReferenceGenerator::TemporalReference ref[] = {
            {0,    (ReferenceGenerator::ReferenceVector() << 0,0,0,0,0,0,1).finished()},
            {10,   (ReferenceGenerator::ReferenceVector() << 1,0,0,0,0,0,1).finished()},
            {20,   (ReferenceGenerator::ReferenceVector() << 0,2,0,0,0,0,1).finished()}
        };
        Fly::Fly()
        : referenceGenerator(ref, countof(ref))
        {
            os::startTime();
        }
    }
}
