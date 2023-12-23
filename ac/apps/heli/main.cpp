#include <errno.h>
#include <stdlib.h>
#include <wirish/wirish.h>

#include <syrup/comm/i2c.hpp>
#include <syrup/math/math.hpp>
#include <syrup/isr.hpp>

#include <ac/com/Serial.hpp>
#include <sys/com/MapleMessages.hpp>
#include <libmaple/i2c.h>
#include <ac/types.hpp>

#include <syrup/drivers/sensors/MPU6050.hpp>
#include <syrup/drivers/sensors/MS5611.hpp>
#include <syrup/drivers/sensors/HMC5883L.hpp>
#include <syrup/drivers/sensors/ADNS3080.hpp>
#include <syrup/drivers/sensors/TCS230.hpp>
#include <syrup/drivers/sensors/Button.hpp>
#include <syrup/drivers/sensors/SRF04.hpp>
#include <syrup/drivers/sensors/AnalogSensor.hpp>

void start_i2c(i2c_dev* dev) {
    i2c_master_enable(dev, I2C_FAST_MODE | I2C_BUS_RESET);
}

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
    pinMode(BOARD_LED_PIN, OUTPUT);
    I2C1->error_callback = start_i2c;
    start_i2c(I2C1);
}

sys::maple::SensorMessage message;

using namespace syrup;
using namespace os;
using namespace sys;
using namespace sys::maple;
U16 ioctl;

HardwareTimer AUXSERVO_TIMER(1);
HardwareTimer SERVO_TIMER(2);
HardwareTimer TIME_TIMER(3);

MPU6050 IMU(I2C1, 17);
MS5611 pressureSensor(I2C1);
HMC5883L magnetometer(I2C1, 18);
ADNS3080 opticalFlow(&Spi2);
SRF04 groundDistance[4] = {{4, Timer3, true},{5, Timer3},{6, Timer3},{7, Timer3}};
TCS230 RPMSensor(19, TIME_TIMER); // Verify args

Button<6>  btn0;
Button<7>  btn1;
Button<13> btn2;
Button<14> btn3;

AnalogSensor powerSensor(3);
AnalogSensor windSensor[2] = {{4}, {5}};

static const U8 motionServo[4] = {8,9,10,11};
static const U8 motor = 25;
static const U8 cameraServo[2] = {26,27};
static const U8 SEL_PIN = 14;

ComputerLink computer(&Serial3);

void timerMidwayIsr() {
    pressureSensor.startPressureSample();
}

void timer10msIsr() {
    static int N = 0;
    //if(!(N % 5)) {
        //for(int i = 0; i < 4; ++i) {
            //groundDistance[i].measure(&message.distance[i]);
        //}
    //}

    IMU.measure(message.imu, &message.nofImu);
    pressureSensor.measure(&message.pressure);
    magnetometer.measure(message.magnetometer, &message.nofMagnetometer);
     message.buttons = (btn3.pressed() << 3)
                     | (btn2.pressed() << 2)
                     | (btn1.pressed() << 1)
                     | (btn0.pressed() << 0);

     windSensor[0].measure(&message.wind[0]);
     windSensor[1].measure(&message.wind[1]);

     powerSensor.measure(&message.power);
     RPMSensor.measure(&message.rpm);

    if(ioctl & IoctlMessage::SEND_SENSOR_DATA) {
        computer.send<>(message);
    }

    memset(&message, 0, sizeof(message));
    pressureSensor.startTemperatureSample();

    N = (N+1)%100;
}


// Run every 5ms
void timerIsr(void*) {
    static int N = 0;
    i2c_wait_for_state_change(I2C1, I2C_STATE_XFER_DONE, 0);

    if (N%2) { // 10 ms
        timer10msIsr();
    } else { // midway
        timerMidwayIsr();
    }

    // Wrap each second
    N = (N+1) % 200;
}


void actuateControl(const U8* msg, const std::size_t len) {
    static ControlMessage m;
    if(len == sizeof(ControlMessage)) {
        memcpy(&m, msg, sizeof(m));

        //~ delay(500);
        pwmWrite(motionServo[0], m.servo[0]);
        pwmWrite(motionServo[1], m.servo[1]);
        pwmWrite(motionServo[2], m.servo[2]);
        pwmWrite(motionServo[3], m.servo[3]);
        pwmWrite(motor, m.rpm);
        if(ioctl & IoctlMessage::RESPONSETEST) {
            computer.send<>(m); // Respond for test application
        }
    }
}

void actuateCamera(const U8* msg, const std::size_t len) {
    CameraControlMessage m;
    if(len == sizeof(CameraControlMessage)) {
        memcpy(&m, msg, sizeof(m));
        pwmWrite(cameraServo[0], m.horizontal);
        pwmWrite(cameraServo[1], m.vertical);
    }
}

void stresstest(void*) {
    for(int i = 0; i < 1000; ++i) {
        computer.send<>(message);
    }
    ioctl ^= IoctlMessage::STRESSTEST;
}

void setIoctl(const U8* msg, const std::size_t) {
    ioctl = bytestou16(msg);

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

    void rxCallback(dma_message*, dma_irq_cause) {
        isr::queue(Computer::readBytes);
    }

    void setup() {
        Serial3.setup_dma();
        computer.device.rxCallback = Computer::rxCallback;
        computer.registerPackager<Messages::controlMessage>(actuateControl);
        computer.registerPackager<Messages::cameraControlMessage>(actuateCamera);
        computer.registerPackager<Messages::ioctlMessage>(setIoctl);
        isr::queue(Computer::readBytes);
    }
}

void timerInterrupt() {isr::queue(timerIsr);}
void timerSetup() {
    TIME_TIMER.pause();
    TIME_TIMER.setPeriod(5000);
    auto ovf = TIME_TIMER.getOverflow();
    TIME_TIMER.setMode(TIMER_CH3, TIMER_OUTPUT_COMPARE);
    TIME_TIMER.setCompare(TIMER_CH3, ovf);
    TIME_TIMER.attachInterrupt(TIMER_CH3, &timerInterrupt);
    TIME_TIMER.refresh();
    TIME_TIMER.resume();
}

void servoSetup() {
    SERVO_TIMER.pause();
    AUXSERVO_TIMER.pause();

    pinMode(SEL_PIN, OUTPUT);
    digitalWrite(SEL_PIN, 1);

    SERVO_TIMER.setPrescaleFactor(11);
    SERVO_TIMER.setOverflow(65455);  // PWM limit: 6546 for 1ms
    SERVO_TIMER.refresh();
    AUXSERVO_TIMER.setPrescaleFactor(11);
    AUXSERVO_TIMER.setOverflow(65455);  // PWM limit: 6546 for 1ms
    AUXSERVO_TIMER.refresh();

    pinMode(motionServo[0], PWM);
    pinMode(motionServo[1], PWM);
    pinMode(motionServo[2], PWM);
    pinMode(motionServo[3], PWM);
    pinMode(cameraServo[0], PWM);
    pinMode(cameraServo[1], PWM);
    pinMode(motor, PWM);

    pwmWrite(motionServo[0], 9818); // 6456*1.5, middle position
    pwmWrite(motionServo[1], 9818);
    pwmWrite(motionServo[2], 9818);
    pwmWrite(motionServo[3], 9818);
    pwmWrite(cameraServo[0], 9818);
    pwmWrite(cameraServo[1], 9818);
    pwmWrite(motor, 0);

    SERVO_TIMER.resume();
    AUXSERVO_TIMER.resume();
}

int main(void) {
    Computer::setup();
    timerSetup();
    servoSetup();
    isr::serviceLoop();
    while(1) asm volatile ("nop");
    return 0;
}
