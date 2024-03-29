#pragma once

#include <os/types.hpp>

namespace sys {
    namespace maple {
        using namespace os;
        struct SensorMessage;
        struct ControlMessage;
        struct CameraControlMessage;
        struct IoctlMessage;
        struct ErrorMessage;
        template<int ID> struct Message;
        struct Messages {
            static const int numberOfMessages = 5;

            enum Id {
                errorMessage = 0,
                sensorMessage = 1,
                controlMessage = 2,
                cameraControlMessage = 3,
                ioctlMessage = 4
            };
            template<Id MID>
            struct ById { typedef typename Message<MID>::Type Type; };

            typedef maple::SensorMessage SensorMessage;
            typedef maple::ControlMessage ControlMessage;
            typedef maple::CameraControlMessage CameraControlMessage;
            typedef maple::ErrorMessage ErrorMessage;
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
            U16 servo[4];
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

        struct ErrorMessage {
            static const Messages::Id ID = Messages::errorMessage;
            U16 type;
            U32 error;
            enum Type {
                I2C = (1 << 0),
            };
        };

        template<> struct Message<Messages::sensorMessage> { typedef SensorMessage Type; };
        template<> struct Message<Messages::controlMessage> { typedef ControlMessage Type; };
        template<> struct Message<Messages::cameraControlMessage> { typedef CameraControlMessage Type; };
        template<> struct Message<Messages::ioctlMessage> { typedef IoctlMessage Type; };
    }
}

