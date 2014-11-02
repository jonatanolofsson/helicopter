#pragma once
#ifndef SYS_COM_MAPLEMESSAGES_HPP_
#define SYS_COM_MAPLEMESSAGES_HPP_

#include <os/types.hpp>

namespace sys {
    namespace maple {
        using namespace os;
        struct SensorMessage;
        struct ControlMessage;
        struct CameraControlMessage;
        struct IoctlMessage;
        template<int ID> struct Message;
        struct Messages {
            static const int numberOfMessages = 4;

            enum Id {
                sensorMessage = 0,
                controlMessage = 1,
                cameraControlMessage = 2,
                ioctlMessage = 3
            };
            template<Id MID>
            struct ById { typedef typename Message<MID>::Type Type; };

            typedef maple::SensorMessage SensorMessage;
            typedef maple::ControlMessage ControlMessage;
            typedef maple::CameraControlMessage CameraControlMessage;
            typedef maple::IoctlMessage IoctlMessage;
        };

        struct SensorMessage {
            static const Messages::Id ID = Messages::sensorMessage;
            enum AccelerometerStates {
                ax = 0,
                ay = 1,
                az = 2,
                wx = 3,
                wy = 4,
                wz = 5
            };
            enum MagnetometerStates {
                mx = 0,
                my = 1,
                mz = 2
            };
            S32 imu[6];
            S32 pressure;
            S32 magnetometer[3];
            //U32 wind[2];
            //U32 distance[4];
            U16 rpm;
            U16 nofImu;
            U16 nofMagnetometer;
            //U16 nofWind[2];
            //U16 nofDistance[4];
            U8 buttons;
        };

        struct ControlMessage {
            static const Messages::Id ID = Messages::controlMessage;
            U16 servo[3];
            U16 rpm;
        };

        struct CameraControlMessage {
            static const Messages::Id ID = Messages::cameraControlMessage;
            U16 horizontal;
            U16 vertical;
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
        template<> struct Message<Messages::cameraControlMessage> { typedef CameraControlMessage Type; };
        template<> struct Message<Messages::ioctlMessage> { typedef IoctlMessage Type; };
    }
}
#endif
