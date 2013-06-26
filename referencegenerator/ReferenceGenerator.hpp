#pragma once
#ifndef SYS_REFERENCEGENERATOR_HPP_
#define SYS_REFERENCEGENERATOR_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/MotionControl.hpp>
#include <sys/ReferenceGenerator.hpp>

namespace sys {
    namespace referencegenerator {
        template<typename Reference>
        struct ReferenceWithTime {
            int time;
            Reference reference;
        };

        template<typename ModelDescription, typename Trigger>
        class ReferenceGenerator {
            public:
                typedef ReferenceGenerator<ModelDescription, Trigger> Self;
                typedef typename ModelDescription::Reference Reference;
                typedef typename ModelDescription::ReferenceMessage ReferenceMessage;
                typedef ReferenceWithTime<Reference> TemporalReference;

            private:
                void yieldReference(const Trigger t);

                const TemporalReference*const reference;
                unsigned nofReferences, i;

                os::Dispatcher<Self, Trigger> d;

            public:
                explicit ReferenceGenerator(const TemporalReference*const ref, unsigned N);

        };
    }
}
#endif
