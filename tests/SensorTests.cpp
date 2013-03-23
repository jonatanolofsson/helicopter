#include <gtest/gtest.h>
#include <os/com/SerialCommunication.hpp>
#include <sys/com/MapleMessages.hpp>
#include <sys/com/MotionControlSignal.hpp>
#include <sys/com/CameraControlSignal.hpp>
#include <os/bytemagic.hpp>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <iomanip>
#include <sys/types.hpp>
#include <termios.h>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/array.hpp>

using namespace sys;

std::condition_variable responseCondition;
std::mutex responseGuard;
int receivedResponse = 0;
bool printResponse = false;

void sensorResponseHandler(const U8* msg, const std::size_t len) {
    ASSERT_NE(msg, nullptr);
    ASSERT_EQ(len, sizeof(SensorMessage));
    ++receivedResponse;
    if(receivedResponse >= 1000) {
        responseCondition.notify_all();
    }
    SensorMessage* m = (SensorMessage*)msg;
    EXPECT_TRUE(m);
    if(printResponse) {
        std::cout
            << std::setw(8) << m->imu[0]
            << std::setw(8) << m->imu[1]
            << std::setw(8) << m->imu[2]
            << std::setw(8) << m->imu[3]
            << std::setw(8) << m->imu[4]
            << std::setw(8) << m->imu[5]
            << std::endl;
    }
}

class SensorTests : public ::testing::Test {
    public:
        typedef SerialCommunication<MapleMessages, 50, 10, B460800> Serial;
        Serial maple;
        SensorTests()
        : maple("/dev/ttyUSB0") // FIXME?
        {
            maple.registerPackager<MapleMessages::sensorMessage>(sensorResponseHandler);

            receivedResponse = 0;
            printResponse = false;
        }

        ~SensorTests() {
            IoctlMessage ioctlMsg = { 0 };
            maple.send<>(ioctlMsg);
        }
};

TEST_F(SensorTests, GetSensorData) {
    IoctlMessage ioctlMsg = { IoctlMessage::SEND_SENSOR_DATA };
    maple.send<>(ioctlMsg);

    //~ printResponse = true;

    std::unique_lock<std::mutex> l(responseGuard);
    responseCondition.wait_for(l, std::chrono::milliseconds(1000), [](){return (receivedResponse > 10);});
    EXPECT_GE(receivedResponse, 10);
}

TEST_F(SensorTests, StressTest) {
    IoctlMessage ioctlMsg = { IoctlMessage::STRESSTEST };
    maple.send<>(ioctlMsg);

    std::unique_lock<std::mutex> l(responseGuard);
    responseCondition.wait_for(l, std::chrono::milliseconds(5000), [](){return (receivedResponse >= 1000);});
    EXPECT_GE(receivedResponse, 1000);
}
