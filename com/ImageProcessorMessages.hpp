#ifndef SYS_COM_IMAGEPROCESSORMESSAGES_HPP_
#define SYS_COM_IMAGEPROCESSORMESSAGES_HPP_

#include <os/types.hpp>

namespace sys {
    using namespace os;
    template<int ID> struct ImageProcessorMessage;
    struct ImageProcessorMessages {
        static const int numberOfMessages = 3;

        enum Id {
            cameraMessage = 0,
            gpsMessage = 1,
            ipCtlMessage = 2
        };
        template<Id MID>
        struct Message { typedef typename MapleMessage<MID>::Type Type; };
    };

    struct CameraMessage {
        static const ImageProcessorMessages::Id ID = ImageProcessorMessages::cameraMessage;
        U16 timestamp;
        U16 position[3];
        U16 orientation[4];
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & timestamp;
            ar & position;
            ar & orientation;
        }
    };

    struct GpsMessage {
        static const ImageProcessorMessages::Id ID = ImageProcessorMessages::gpsMessage;
        U16 timestamp;
        U16 position[3];
        U16 velocity[3];
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & timestamp;
            ar & position;
            ar & velocity;
        }
    };
    struct IpCtlMessage {
        static const ImageProcessorMessages::Id ID = ImageProcessorMessages::ipCtlMessage;
        U16 message;
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & message;
        }
    };

    template<> struct ImageProcessorMessage<ImageProcessorMessages::cameraMessage> { typedef CameraMessage Type; };
    template<> struct ImageProcessorMessage<ImageProcessorMessages::gpsMessage> { typedef GpsMessage Type; };
    template<> struct ImageProcessorMessage<ImageProcessorMessages::ipCtlMessage> { typedef IpCtlMessage Type; };
}
#endif
