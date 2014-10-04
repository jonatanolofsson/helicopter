#pragma once
#ifndef REFERENCEGENERATOR_HPP_
#define REFERENCEGENERATOR_HPP_

#include <sys/types.hpp>
#include <sys/referencegenerator/ReferenceGenerator.hpp>

namespace sys {
    namespace referencegenerator {
        typedef motioncontrol::ModelDescription ModelDescription;
        typedef os::SystemTime Trigger;
    }
    
    typedef referencegenerator::ReferenceGenerator<referencegenerator::ModelDescription, referencegenerator::Trigger> ReferenceGenerator;
}

#endif
