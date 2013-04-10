#pragma once
#ifndef SYS_COM_STMMESSAGES_HPP_
#define SYS_COM_STMMESSAGES_HPP_

#include <os/types.hpp>

namespace sys {
    namespace stm {
        using namespace os;
        template<int ID> struct Message;
        struct Messages {
            static const int numberOfMessages = 7;

            enum Id {
                sensorMessage = 0,
                controlMessage = 1,
                irCameraMessage = 2,
                towerMessage = 3,
                waterMessage = 4,
                fftMessage = 5,
                ioctlMessage = 6
            };
            template<Id MID>
            struct ById { typedef typename Message<MID>::Type Type; };
        };

        struct SensorMessage {
            static const Messages::Id ID = Messages::sensorMessage;
            U16 distance[4];
        };

        struct ControlMessage {
            static const Messages::Id ID = Messages::controlMessage;
            S16 l;
            S16 r;
        };

        struct TowerMessage {
            static const Messages::Id ID = Messages::towerMessage;
            S16 angle;
        };

        struct WaterMessage {
            static const Messages::Id ID = Messages::waterMessage;
            U8 spray;
        };

        struct FFTMessage {
            static const Messages::Id ID = Messages::fftMessage;
            U8 active;
        };

        struct IrCameraMessage {
            static const Messages::Id ID = Messages::irCameraMessage;
            U16 blobs[4][2];
        };

        struct IoctlMessage {
            static const Messages::Id ID = Messages::ioctlMessage;
            U16 message;
            enum Bits {
                SEND_SENSOR_DATA = (1 << 0),
                RESPONSETEST = (1 << 1),
                STRESSTEST = (1 << 2)
            };
        };

        template<> struct Message<Messages::sensorMessage> { typedef SensorMessage Type; };
        template<> struct Message<Messages::controlMessage> { typedef ControlMessage Type; };
        template<> struct Message<Messages::irCameraMessage> { typedef IrCameraMessage Type; };
        template<> struct Message<Messages::towerMessage> { typedef TowerMessage Type; };
        template<> struct Message<Messages::waterMessage> { typedef WaterMessage Type; };
        template<> struct Message<Messages::fftMessage> { typedef FFTMessage Type; };
        template<> struct Message<Messages::ioctlMessage> { typedef IoctlMessage Type; };
    }
}
#endif
