#pragma once
#ifndef SYS_REFERENCEGENERATOR_IMPLEMENTATION_HPP_
#define SYS_REFERENCEGENERATOR_IMPLEMENTATION_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/settings.hpp>

namespace sys {
    namespace referencegenerator {
        template<typename ModelDescription, typename Trigger>
        ReferenceGenerator<ModelDescription, Trigger>::ReferenceGenerator(const TemporalReference*const ref, unsigned N)
        : reference(ref)
        , nofReferences(N)
        , i(0)
        , d(&Self::yieldReference, this)
        {}

        template<typename ModelDescription, typename Trigger>
        void ReferenceGenerator<ModelDescription, Trigger>::yieldReference(const Trigger t) {
            while((t.value > reference[i].time) && ((i+1) < nofReferences)) ++i;
            os::yield(ReferenceMessage(reference[i].reference));
        }
    }
}

#endif
