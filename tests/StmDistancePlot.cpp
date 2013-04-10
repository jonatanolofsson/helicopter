#include <os/com/SerialCommunication.hpp>
#include <sys/com/Stm.hpp>
#include <os/bytemagic.hpp>
#include <iomanip>
#include <sys/types.hpp>
#include <termios.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <endian.h>

using namespace sys;
using namespace sys::stm;


#include <cpplot/cpplot.hpp>
using namespace cpplot;

template<int rows, int cols>
int index(int row, int col) { return row + (col-1)*rows; }

static const int count = 30*10;
#define rows 2
#define cols 2
#define plotname "Distance"

auto awin = figure(plotname)->subplot(rows,cols,index<rows,cols>(1,1))->title("1");
auto aplot = awin->add<Line>()->set("r")->set_capacity(count);
auto bwin = figure(plotname)->subplot(rows,cols,index<rows,cols>(1,2))->title("2");
auto bplot = bwin->add<Line>()->set("r")->set_capacity(count);
auto cwin = figure(plotname)->subplot(rows,cols,index<rows,cols>(2,1))->title("3");
auto cplot = cwin->add<Line>()->set("r")->set_capacity(count);
auto dwin = figure(plotname)->subplot(rows,cols,index<rows,cols>(2,2))->title("4");
auto dplot = dwin->add<Line>()->set("r")->set_capacity(count);

void sensorResponseHandler(const U8* msg, const std::size_t) {
    //~ std::cout << "Got message" << std::endl;
    SensorMessage* m = (SensorMessage*)msg;
    static int t = 0; ++t;
    aplot << std::make_pair(t, (double)14.0);
    bplot << std::make_pair(t, (S16)le16toh(m->distance[1]));
    //~ cplot << std::make_pair(t, (S16)le16toh(m->distance[2]));
    //~ dplot << std::make_pair(t, (S16)le16toh(m->distance[3]));


    //~ std::cout
        //~ << std::setw(8) << (float)(S16)le16toh(m->distance[0])
        //~ << std::setw(8) << (S16)le16toh(m->distance[1])
        //~ << std::setw(8) << (S16)le16toh(m->distance[2])
        //~ << std::setw(8) << (S16)le16toh(m->distance[3])
        //~ << std::endl;
}


int main(int argc, char* argv[]){
    cpplot::glut::init(argc, argv);

    //~ typedef SerialCommunication<stm::Messages, 50, 10, B460800> Serial;
    typedef SerialCommunication<stm::Messages, 50, 10, B115200> Serial;
    Serial stm("/dev/ttyUSB0");
    stm.registerPackager<stm::Messages::sensorMessage>(sensorResponseHandler);

    {
        ControlMessage ctlMsg[4] = {
            { 500, 500 },
            { -500, 500 },
            { -500, -500 },
            { 500, -500 }
        };

        int i = 0;
        while(true) {
            stm.send<>(ctlMsg[i++%4]);
            sleep(1);
        }
    }

    std::cout << "Wait for messages.." << std::endl;

    while(true) std::this_thread::sleep_for(std::chrono::seconds(60));

    return 0;
}
