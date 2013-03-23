#include <errno.h>
#include <stdlib.h>
#include <wirish/wirish.h>

void print(int v, int base = 10);
void print(int v, int base) {
    if (v < 0) {
        Serial3.write('-');
        v = -v;
    }
    if (v >= base) {
        print(v/base, base);
    }
    int p = v%base;
    if(p < 10) {
        Serial3.write('0'+p);
    } else {
        Serial3.write('A'+p);
    }
}


#include <syrup/comm/i2c.hpp>
#include <syrup/isr.hpp>
//~
#include "Serial.hpp"
#include <libmaple/i2c.h>
#include <Servo/Servo.h>
#include "types.hpp"
//~
#include <syrup/drivers/sensors/MPU6050.hpp>
#include <syrup/drivers/sensors/MS5611.hpp>
#include <syrup/drivers/sensors/HMC5883L.hpp>
//~ #include <syrup/drivers/sensors/TCS230.hpp>
//~ #include <syrup/drivers/sensors/Button.hpp>
//~ #include <syrup/drivers/sensors/SRF04.hpp>
//~ #include <syrup/drivers/sensors/AnalogSensor.hpp>
//~
//~ #define WATCHDOG                (12)


// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
    pinMode(BOARD_LED_PIN, OUTPUT);
    //~ Serial3.begin(115200);

    i2c_master_enable(I2C1, I2C_FAST_MODE | I2C_BUS_RESET);
}

sys::SensorMessage message;

using namespace syrup;
using namespace os;
using namespace sys;
U16 ioctl;

MPU6050 IMU(I2C1, 17);
MS5611 pressureSensor(I2C1);
HMC5883L magnetometer(I2C1, 18);
//~ SRF04 groundDistance[4] = {{4, Timer3, true},{5, Timer3},{6, Timer3},{7, Timer3}};
//~ TCS230 RPMSensor(19, Timer4); // Verify args
//~
//~ Button<6>  btn0;
//~ Button<7>  btn1;
//~ Button<13> btn2;
//~ Button<14> btn3;
//~ //~
//~ AnalogSensor powerSensor(3);
//~ AnalogSensor windSensor[2] = {{4}, {5}};
//~
//~ Servo motionServo[3] = {{24},{25},{26}};
//~ Servo motor = {27};
//~ Servo cameraServo[2] = {{10},{11}};

ComputerLink computer(&Serial3);

void timerISR(void*) {
    static int N = 0;

    //~ digitalWrite(BOARD_LED_PIN, N%2);

    // Feed the watchdog
    //~ digitalWrite(WATCHDOG, (N % 5) == 0);

    //~ pressureSensor.sample();

    //~ if(!(N % 5)) {
        //~ for(int i = 0; i < 4; ++i) {
            //~ groundDistance[i].measure(&message.distance[i]);
        //~ }
    //~ }

    IMU.measure(message.imu);
    //~ digitalWrite(BOARD_LED_PIN, message.imu[2] != 0);
    //~ pressureSensor.measure(&message.pressure);
    magnetometer.measure(message.magnetometer);
    //~ message.buttons = (btn3.pressed() << 3)
                    //~ | (btn2.pressed() << 2)
                    //~ | (btn1.pressed() << 1)
                    //~ | (btn0.pressed() << 0);

    //~ windSensor[0].measure(&message.wind[0]);
    //~ windSensor[1].measure(&message.wind[1]);
    //~ Serial3.println('a');
    //~ for(int i = 0; i < 3; ++i) {
        //~ print((int)message.imu[i]);
        //~ Serial3.print("        ");
    //~ }
    //~ Serial3.println("");

    //~ //~// powerSensor.measure(&message.power);
    //~ RPMSensor.measure(&message.rpm);

    if(ioctl & IoctlMessage::SEND_SENSOR_DATA) {
        //~ digitalWrite(BOARD_LED_PIN, N%2);
        computer.send<>(message);
    }

    // Wrap each second
    N = (N+1) % 100;
}


void actuateControl(const U8* msg, const std::size_t len) {
    //~ static bool a = true; a = !a;
    //~ digitalWrite(BOARD_LED_PIN, a);
    if(len == sizeof(sys::ControlMessage)) {
        sys::ControlMessage *m = (sys::ControlMessage*)msg; // FIXME
        //~ digitalWrite(BOARD_LED_PIN, a);
        //~ a = !a;
        //~ delay(500);
        //~ motionServo[0].set(m->servo[0]);
        //~ motionServo[1].set(m->servo[1]);
        //~ motionServo[2].set(m->servo[2]);
        //~ motor.set(m->rpm);
        if(ioctl & IoctlMessage::RESPONSETEST) {
            computer.send<>(*m); // Respond for test application
        }
    }
}
void actuateCamera(const U8* msg, const std::size_t len) {
    if(len == sizeof(sys::CameraControlMessage)) {
        sys::CameraControlMessage *m = (sys::CameraControlMessage*)msg; // Safe because of requested alignment of msg
        //~ cameraServo[0].set(m->horizontal);
        //~ cameraServo[1].set(m->vertical);
    }
}

void stresstest(void*) {
    for(int i = 0; i < 1000; ++i) {
        computer.send<>(message);
    }
    ioctl ^= IoctlMessage::STRESSTEST;
}

void setIoctl(const U8* msg, const std::size_t len) {
    ioctl = *(U16*)msg;
    digitalWrite(BOARD_LED_PIN, 1);

    if(ioctl & IoctlMessage::STRESSTEST) {
        isr::queue(stresstest);
    }
}

namespace Computer {
    void readBytes(void*) {
        if(computer.readBytes() == -EAGAIN) {
            isr::queue(Computer::readBytes);
        }
    }
    void rxCallback(dma_message*, dma_irq_cause cause) {
        isr::queue(Computer::readBytes);
    }
    void setup() {
        Serial3.setup_dma();
        computer.device.rxCallback = Computer::rxCallback;
        computer.registerPackager<MapleMessages::controlMessage>(actuateControl);
        computer.registerPackager<MapleMessages::cameraControlMessage>(actuateCamera);
        computer.registerPackager<MapleMessages::ioctlMessage>(setIoctl);
        isr::queue(Computer::readBytes);
    }
}

void timer10msInterrupt() {
    //~ static bool a = true; a = !a;
    //~ digitalWrite(BOARD_LED_PIN, 1);
    isr::queue(timerISR);
}
void timerSetup() {
    Timer4.setPeriod(10000);
    Timer4.attachInterrupt(1, &timer10msInterrupt);
    Timer4.refresh();
}

int main(void) {
    Computer::setup();
    timerSetup();
    isr::serviceLoop();
    while(1);
    return 0;
}
