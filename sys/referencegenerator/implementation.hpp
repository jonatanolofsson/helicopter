#pragma once
#ifndef SYS_REFERENCEGENERATOR_IMPLEMENTATION_HPP_
#define SYS_REFERENCEGENERATOR_IMPLEMENTATION_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/settings.hpp>
#include <os/utils/eventlog.hpp>

namespace sys {
    namespace referencegenerator {
        template<typename ReferenceMessage, typename Trigger>
        ReferenceGenerator<ReferenceMessage, Trigger>::ReferenceGenerator(const TemporalReference*const ref, unsigned N)
        : reference(ref), nofReferences(N), i(0), d(&Self::yieldReference, this)
        {}

        template<typename ReferenceMessage, typename Trigger>
        void ReferenceGenerator<ReferenceMessage, Trigger>::yieldReference(const Trigger t) {
            while(((i+1) < nofReferences) && (t.value >= reference[i+1].time)) ++i;
            LOG_EVENT(typeid(Self).name(), 0, "Reference at time " << t.value);
            os::yield(ReferenceMessage(reference[i].reference));
        }
    }
}

#endif
