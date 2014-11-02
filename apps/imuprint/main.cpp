#include <os/com/SerialCommunication.hpp>
#include <sys/com/Maple.hpp>
#include <os/bytemagic.hpp>
#include <iomanip>
#include <sys/types.hpp>
#include <termios.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <endian.h>

using namespace sys;
using namespace sys::maple;


#include <cpplot/cpplot.hpp>
using namespace cpplot;

template<int rows, int cols>
int index(int row, int col) { return row + (col-1)*rows; }

static const int count = 30*10;
#define rows 4
#define cols 1
#define plotname "Imu"

auto awin = figure(plotname)->subplot(rows,cols,index<rows,cols>(1,1))->title("Acc");
auto axplot = awin->add<Line>()->set("r")->set_capacity(count);
auto ayplot = awin->add<Line>()->set("g")->set_capacity(count);
auto azplot = awin->add<Line>()->set("b")->set_capacity(count);
auto wwin = figure(plotname)->subplot(rows,cols,index<rows,cols>(2,1))->title("Rotvel");
auto wxplot = wwin->add<Line>()->set("r")->set_capacity(count);
auto wyplot = wwin->add<Line>()->set("g")->set_capacity(count);
auto wzplot = wwin->add<Line>()->set("b")->set_capacity(count);
auto mwin = figure(plotname)->subplot(rows,cols,index<rows,cols>(3,1))->title("Orientation");
auto mxplot = mwin->add<Line>()->set("r")->set_capacity(count);
auto myplot = mwin->add<Line>()->set("g")->set_capacity(count);
auto mzplot = mwin->add<Line>()->set("b")->set_capacity(count);
auto altwin = figure(plotname)->subplot(rows,cols,index<rows,cols>(4,1))->title("Altitude");
auto altplot = altwin->add<Line>()->set("b")->set_capacity(count);

double getAltitude(double pressure, double seaLevelPressure=101325)
{
    return (44330.0 * (1.0 - pow((double)pressure / (double)seaLevelPressure, 0.1902949)));
}

void sensorResponseHandler(const U8* msg, const std::size_t) {
    //std::cout << "Got message" << std::endl;
    SensorMessage m = os::fromBytes<SensorMessage>(msg);
    double imu[6];
    double mag[3];
    double alt;
    for(int i = 0; i < 6; ++i) {
        imu[i] = (m.nofImu ? ((double)m.imu[i]) / ((double)m.nofImu) : 0);
    }
    for(int i = 0; i < 3; ++i) {
        mag[i] = (m.nofMagnetometer ? ((double)m.magnetometer[i]) / ((double)m.nofMagnetometer) : 0);
    }
    alt = getAltitude(m.pressure);
    static double oalt = alt;
    double alpha = 0.01;
    auto altfilt = (1-alpha)*oalt + alpha*alt;
    oalt = altfilt;

    static int t = 0; ++t;
    axplot << std::make_pair(t, imu[0]);
    ayplot << std::make_pair(t, imu[1]);
    azplot << std::make_pair(t, imu[2]);
    wxplot << std::make_pair(t, imu[3]);
    wyplot << std::make_pair(t, imu[4]);
    wzplot << std::make_pair(t, imu[5]);
    mxplot << std::make_pair(t, mag[0]);
    myplot << std::make_pair(t, mag[1]);
    mzplot << std::make_pair(t, mag[2]);
    altplot << std::make_pair(t, altfilt);




        std::cout
            << std::setw(8) << m.imu[0]
            << std::setw(8) << m.imu[1]
            << std::setw(8) << m.imu[2]
            << std::setw(8) << m.imu[3]
            << std::setw(8) << m.imu[4]
            << std::setw(8) << m.imu[5]
            << std::setw(8) << m.nofImu
            << std::setw(8) << "|"
            << std::setw(8) << m.magnetometer[0]
            << std::setw(8) << m.magnetometer[1]
            << std::setw(8) << m.magnetometer[2]
            << std::setw(8) << m.nofMagnetometer
            << std::setw(8) << "|"
            << std::setw(8) << m.pressure
            << std::setw(8) << alt
            << std::endl;
}


int main(int argc, char* argv[]){
    cpplot::glut::init(argc, argv);

    Maple maple("/dev/ttyUSB0");
    maple.registerPackager<maple::Messages::sensorMessage>(sensorResponseHandler);

    IoctlMessage ioctlMsg = { IoctlMessage::SEND_SENSOR_DATA };
    maple.send<>(ioctlMsg);

    std::cout << "Wait for messages.." << std::endl;

    while(true) std::this_thread::sleep_for(std::chrono::seconds(60));

    return 0;
}
