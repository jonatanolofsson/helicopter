#pragma once
#ifndef SYS_REFERENCEGENERATOR_HPP_
#define SYS_REFERENCEGENERATOR_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/MotionControl.hpp>
#include <sys/ReferenceGenerator.hpp>
#include <os/clock.hpp>

namespace sys {
    namespace referencegenerator {
        template<typename Reference>
        struct ReferenceWithTime {
            os::TimeType time;
            Reference reference;
        };

        template<typename ReferenceMessage, typename Trigger>
        class ReferenceGenerator {
            public:
                typedef ReferenceGenerator<ReferenceMessage, Trigger> Self;
                typedef typename ReferenceMessage::StateVector ReferenceVector;
                typedef ReferenceWithTime<ReferenceVector> TemporalReference;

            private:
                void yieldReference(const Trigger t);

                const TemporalReference*const reference;
                int nofReferences, i;

                os::Dispatcher<Self, Trigger> d;

            public:
                explicit ReferenceGenerator(const TemporalReference*const ref, unsigned N);

        };
    }
}
#endif
