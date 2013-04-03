#include <sys/localplanner/API.hpp>
#include <sys/localplanner/LocalPlanner.hpp>
#include <os/com/getSignal.hpp>
#include <iostream>

INSTANTIATE_SIGNAL(sys::localplanner::Checkpoint);

namespace sys {
    namespace localplanner {
        LocalPlanner::LocalPlanner()
        : d_reference(&Self::yieldReference, this)
        , d_destination(&Self::updateDestination, this)
        {}

        void LocalPlanner::updateDestination(const Checkpoint d) {
            std::unique_lock<std::mutex> l(guard);
            destination = d;
        }

        void LocalPlanner::yieldReference(const observer::SystemState) {
            std::cout << "Localplanner: Yield reference" << std::endl;
            // Conceptual pseudocode below
            //~ Scalar angle;
            //~ {
                //~ std::unique_lock<std::mutex> l(guard);
                //~ angle = angle = closest_angle(position(x), destination);
            //~ }
            //~ if(angle < settings.angleRotateFirst) {
                //~ yield(Controller::Reference(0, angle));
            //~ } else {
                //~ yield(Controller::Reference(v, angle));
            //~ }
        }
    }
}
