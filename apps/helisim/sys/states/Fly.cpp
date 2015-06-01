#include <sys/ReferenceGenerator.hpp>
#include <sys/states/Fly.hpp>
#include <sys/math/base.hpp>
#include <os/utils/params.hpp>
#include <Eigen/Core>

namespace sys {
    namespace states {
        static_assert(ReferenceGenerator::ReferenceVector::RowsAtCompileTime == 9, "Wrong number of elements in reference");
        ReferenceGenerator::TemporalReference ref[] = {
            {0,    (ReferenceGenerator::ReferenceVector() << 0,0,0,0,0,0,0,0,0).finished()},
            //{250,   (ReferenceGenerator::ReferenceVector() << 1,0,0,0,0,1,0,0,0).finished()},
            //{500,   (ReferenceGenerator::ReferenceVector() << 0,2,0,0,0,1,0,0,0).finished()}
        };
        Fly::Fly(my_context ctx)
        : Base(ctx)
        , motionController(context<Top>().globalFilter)
        , referenceGenerator(ref, countof(ref))
        , simulator(context<Top>().globalFilter)
        {
            motionController.initializeFromParams();
            math::models::helicopter::Parameters<>::initializeFromParams();

            simulator.filter.state[simulator::States::N] = os::parameters["Helicopter"]["N"].GetDouble();
            context<Top>().observer.filter.state[simulator::States::N] = os::parameters["Helicopter"]["N"].GetDouble();

            os::startTime();
        }
    }
}
