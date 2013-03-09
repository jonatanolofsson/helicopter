#include <gtest/gtest.h>
#include <os/com/SerialCommunication.hpp>
#include <sys/actuator/API.hpp>
#include <sys/com/MapleMessages.hpp>
#include <sys/com/MotionControlSignal.hpp>
#include <sys/com/CameraControlSignal.hpp>
#include <os/bytemagic.hpp>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <sys/types.hpp>
#include <termios.h>

using namespace sys;
using actuator::Actuator;

std::condition_variable responseCondition;
std::mutex responseGuard;
bool receivedResponse = false;

template<typename T>
void responseHandler(const U8* msg, const std::size_t len) {
    ASSERT_NE(msg, nullptr);
    ASSERT_EQ(len, sizeof(T));
    std::unique_lock<std::mutex> l(responseGuard);
    receivedResponse = true;
    responseCondition.notify_all();
}

class ActuatorTests : public ::testing::Test {
    public:
        typedef SerialCommunication<MapleMessages, 100, 10, B460800> Serial;
        Serial maple;
        Actuator<Serial> actuator;
        ActuatorTests()
        : maple("/dev/ttyUSB0") // FIXME
        , actuator(maple)
        {
            maple.registerPackager<MapleMessages::controlMessage>(responseHandler<ControlMessage>);
            maple.registerPackager<MapleMessages::cameraControlMessage>(responseHandler<CameraControlMessage>);
        }
};

TEST_F(ActuatorTests, SetServoOutput) {
    IoctlMessage ioctlMsg = { IoctlMessage::RESPONSETEST };
    maple.send<>(ioctlMsg);
    receivedResponse = false;

    MotionControlSignal m;

    m[control::servo[0]]    = 0;
    m[control::servo[1]]    = 0;
    m[control::servo[2]]    = 0;
    m[control::rpm]         = 0;
    //~ std::cout << "Sizeof: " << sizeof(m) << std::endl;
    yield(m);
    std::unique_lock<std::mutex> l(responseGuard);
    responseCondition.wait_for(l, std::chrono::milliseconds(300), [](){return receivedResponse;});
    EXPECT_EQ(receivedResponse, true);
}
