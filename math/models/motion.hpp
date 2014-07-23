#pragma once
#ifndef SYS_MATH_MODELS_MOTION_HPP_
#define SYS_MATH_MODELS_MOTION_HPP_

#include <sys/types.hpp>
#include <sys/math/filtering.hpp>
#include <sys/com/EigenMessage.hpp>

namespace sys {
    namespace math {
        namespace models {
            namespace messages {
                template<typename States> struct StateMessage : public EigenMessage<States> { StateMessage(){} explicit StateMessage(const States& s) : EigenMessage<States>(s) {} };
                template<typename Reference> struct ReferenceMessage : public EigenMessage<Reference> { ReferenceMessage(){} explicit ReferenceMessage(const Reference& r) : EigenMessage<Reference>(r) {} };
                template<typename Controls> struct ControlMessage : public EigenMessage<Controls> { ControlMessage(){} explicit ControlMessage(const Controls& c) : EigenMessage<Controls>(c) {} };
            }

            template<typename SDescription, typename CDescription, typename ADescription, typename S = Scalar>
            struct Description {
                typedef S Scalar;
                typedef SDescription StateDescription;
                typedef CDescription ControlDescription;
                typedef ADescription AuxiliaryDescription;

                static const int nofStates      = StateDescription::nofStates;
                static const int nofControls    = ControlDescription::nofControls;
                static const int nofAuxiliaries = ADescription::nofAuxiliaries;

                typedef StateVector<Scalar, nofStates> States;
                typedef States Reference;
                typedef StateVector<Scalar, nofControls> Controls;
                typedef StateVector<Scalar, nofAuxiliaries> Auxiliaries;

                typedef messages::StateMessage<States> StateMessage;
                typedef messages::ReferenceMessage<Reference> ReferenceMessage;
                typedef messages::ControlMessage<Controls> ControlMessage;
            };
        }
    }
}

#include <sys/math/models/motion/ConstantPosition3D.hpp>
#include <sys/math/models/motion/ConstantVelocities3D.hpp>
#include <sys/math/models/motion/ConstantVelocities6D.hpp>
#include <sys/math/models/motion/DirectVelocities3D.hpp>
#include <sys/math/models/motion/CoordinatedTurn2D.hpp>
#include <sys/math/models/motion/CoordinatedTurn2DPose.hpp>
#include <sys/math/models/motion/Helicopter.hpp>
#include <sys/math/models/motion/HelicopterControl.hpp>

#endif
