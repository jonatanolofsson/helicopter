#include <stdlib.h>
#include <wirish/wirish.h>

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {init();}

#include <syrup/comm/i2c.hpp>

#include "Serial.hpp"
#include <Servo/Servo.h>

#include <syrup/drivers/sensors/MPU6050.hpp>
#include <syrup/drivers/sensors/TCS230.hpp>
#include <syrup/drivers/sensors/MS5611.hpp>
#include <syrup/drivers/sensors/HMC5883L.hpp>
#include <syrup/drivers/sensors/Button.hpp>
#include <syrup/drivers/sensors/SRF04.hpp>
#include <syrup/drivers/sensors/AnalogSensor.hpp>

#define WATCHDOG                (12)

sys::SensorMessage message;

using namespace syrup;
using namespace os;
using namespace sys;
U16 ioctl;

HardWire HWI2C(1, I2C_FAST_MODE);
I2CPeripheral i2cMPU6050(&HWI2C, MPU6050::I2C_ADDRESS_LOW);
//~ MPU6050 IMU(&i2cMPU6050, 18);
//~ I2CPeripheral i2cMS5611(&HWI2C, MS5611::I2C_ADDRESS_LOW);
//~ MS5611 pressureSensor(&i2cMS5611, false);
//~ I2CPeripheral i2cHMC5883(&HWI2C, HMC5883L::I2C_ADDRESS);
//~ HMC5883L magnetometer(&i2cHMC5883, 17);
//~ SRF04 groundDistance[4] = {{4, Timer3, true},{5, Timer3},{6, Timer3},{7, Timer3}};
//~ TCS230 RPMSensor(19, Timer4); // Verify args

//~ Button<6>  btn0;
//~ Button<7>  btn1;
//~ Button<13> btn2;
//~ Button<14> btn3;
//~
//~ AnalogSensor powerSensor(3);
//~ AnalogSensor windSensor[2] = {{4}, {5}};
//~
//~ Servo motionServo[3] = {{24},{25},{26}};
//~ Servo motor = {27};
//~ Servo cameraServo[2] = {{10},{11}};

ComputerLink computer(&Serial3);

void timer10msInterrupt() {
    static int N = 0;

    // Feed the watchdog
    //~ digitalWrite(WATCHDOG, (N % 5) == 0);

    //~ pressureSensor.timer_sample();

    //~ if(!(N % 5)) {
        //~ for(int i = 0; i < 4; ++i) {
            //~ groundDistance[i].measure(&message.distance[i]);
        //~ }
    //~ }

    //~ IMU.measure(message.imu);
    //~ pressureSensor.measure(&message.pressure);
    //~ magnetometer.measure(message.magnetometer);
    //~ message.buttons = (btn3.pressed() << 3)
                    //~ | (btn2.pressed() << 2)
                    //~ | (btn1.pressed() << 1)
                    //~ | (btn0.pressed() << 0);

    //~ windSensor[0].measure(&message.wind[0]);
    //~ windSensor[1].measure(&message.wind[1]);

    //~ //~// powerSensor.measure(&message.power);
    //~ RPMSensor.measure(&message.rpm);

    //~ if(ioctl & IoctlMessage::SEND_SENSOR_DATA) {
        //~ computer.send<>(message);
    //~ }

    // Wrap each second
    N = (N+1) % 100;
}


void actuateControl(const U8* msg, const std::size_t len) {
    //~ static bool a = true;
    if(len == sizeof(sys::ControlMessage)) {
        sys::ControlMessage *m = (sys::ControlMessage*)msg; // Safe because of requested alignment of msg
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
    //~ if(len == sizeof(sys::CameraControlMessage)) {
        //~ sys::CameraControlMessage *m = (sys::CameraControlMessage*)msg; // Safe because of requested alignment of msg
        //~ cameraServo[0].set(m->horizontal);
        //~ cameraServo[1].set(m->vertical);
    //~ }
}
void setIoctl(const U8* msg, const std::size_t len) {
    ioctl = *(U16*)msg;

    if(ioctl & IoctlMessage::STRESSTEST) {
        //~ digitalWrite(BOARD_LED_PIN, 1);
        for(int i = 0; i < 1000; ++i) {
            computer.send<>(message);
        }
        ioctl ^= IoctlMessage::STRESSTEST;
        //~ digitalWrite(BOARD_LED_PIN, 0);
    }
}

void computerSetup() {
    Serial3.setup_dma_tx();
    computer.registerPackager<MapleMessages::controlMessage>(actuateControl);
    computer.registerPackager<MapleMessages::cameraControlMessage>(actuateCamera);
    computer.registerPackager<MapleMessages::ioctlMessage>(setIoctl);
}

void timerSetup() {
    Timer4.setPeriod(10000);
    Timer4.attachInterrupt(1, &timer10msInterrupt);
    Timer4.refresh();
}

int main(void) {
    pinMode(BOARD_LED_PIN, OUTPUT);
    computerSetup();
    //~ timerSetup();
    computer.readerLoop();
    return 0;
}
