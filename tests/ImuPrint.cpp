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
#define rows 2
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

void sensorResponseHandler(const U8* msg, const std::size_t) {
    //std::cout << "Got message" << std::endl;
    SensorMessage* m = (SensorMessage*)msg;
    double imu[6];
    for(int i = 0; i < 6; ++i) {
        imu[i] = (m->nofImu ? ((double)m->imu[i]) / ((double)m->nofImu) : 0);
    }
    
    static int t = 0; ++t;
    axplot << std::make_pair(t, imu[0]);
    ayplot << std::make_pair(t, imu[1]);
    azplot << std::make_pair(t, imu[2]);
    wxplot << std::make_pair(t, imu[3]);
    wyplot << std::make_pair(t, imu[4]);
    wzplot << std::make_pair(t, imu[5]);


        std::cout
            << std::setw(8) << imu[0]
            << std::setw(8) << imu[1]
            << std::setw(8) << imu[2]
            << std::setw(8) << imu[3]
            << std::setw(8) << imu[4]
            << std::setw(8) << imu[5]
            << std::setw(8) << m->nofImu
            << std::endl;
}


int main(int argc, char* argv[]){
    cpplot::glut::init(argc, argv);

    typedef SerialCommunication<maple::Messages, 100, 10, B460800> Serial;
    Serial maple("/dev/ttyUSB0");
    maple.registerPackager<maple::Messages::sensorMessage>(sensorResponseHandler);

    IoctlMessage ioctlMsg = { IoctlMessage::SEND_SENSOR_DATA };
    maple.send<>(ioctlMsg);

    std::cout << "Wait for messages.." << std::endl;

    while(true) std::this_thread::sleep_for(std::chrono::seconds(60));

    return 0;
}
