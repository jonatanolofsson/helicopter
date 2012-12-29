#include <gtest/gtest.h>
#include <os/com/SerialCommunication.hpp>
#include <sys/Actuator.hpp>
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

std::condition_variable responseCondition;
std::mutex responseGuard;
int receivedResponse = 0;

void sensorResponseHandler(const U8* msg, const std::size_t len) {
    ASSERT_EQ(msg > 0, true);
    ASSERT_EQ(len, sizeof(SensorMessage));
    ++receivedResponse;
    if(receivedResponse >= 1000) {
        responseCondition.notify_all();
    }
    SensorMessage* m = (SensorMessage*)msg;
    EXPECT_TRUE(m);
    //~ std::cout << m->imu[0] << std::endl;
}

class SensorTests : public ::testing::Test {
    public:
        typedef SerialCommunication<MapleMessages, 50, 10> Serial;
        Serial maple;
        SensorTests()
        : maple("/dev/ttyUSB0") // FIXME?
        {
            maple.registerPackager<MapleMessages::sensorMessage>(sensorResponseHandler);
        }
};

//~ TEST_F(SensorTests, GetServoData) {
    //~ IoctlMessage ioctlMsg = { IoctlMessage::SEND_SENSOR_DATA };
    //~ maple.send<>(ioctlMsg);
//~
    //~ std::unique_lock<std::mutex> l(responseGuard);
    //~ responseCondition.wait_for(l, std::chrono::milliseconds(3000), [](){return (receivedResponse > 100);});
    //~ EXPECT_GT(receivedResponse, 1000);
//~ }

TEST_F(SensorTests, StressTest) {
    IoctlMessage ioctlMsg = { IoctlMessage::STRESSTEST };
    maple.send<>(ioctlMsg);

    std::unique_lock<std::mutex> l(responseGuard);
    responseCondition.wait_for(l, std::chrono::milliseconds(3000), [](){return (receivedResponse > 100);});
    EXPECT_GE(receivedResponse, 1000);
}
