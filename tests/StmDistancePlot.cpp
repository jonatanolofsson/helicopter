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
#include <os/bytemagic.hpp>

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
auto aplot = awin->add<Line>()->set_capacity(count);
auto bwin = figure(plotname)->subplot(rows,cols,index<rows,cols>(1,2))->title("2");
auto bplot = bwin->add<Line>()->set_capacity(count);
auto cwin = figure(plotname)->subplot(rows,cols,index<rows,cols>(2,1))->title("3");
auto cplot = cwin->add<Line>()->set_capacity(count);
auto dwin = figure(plotname)->subplot(rows,cols,index<rows,cols>(2,2))->title("4");
auto dplot = dwin->add<Line>()->set_capacity(count);

#undef rows
#undef cols
#undef plotname

#define rows 3
#define cols 4
#define plotname "Light"

auto xwin1 = figure(plotname)->subplot(rows,cols,index<rows,cols>(1,1));
auto xplot1 = xwin1->add<Line>()->set_capacity(count);
auto ywin1 = figure(plotname)->subplot(rows,cols,index<rows,cols>(1,2));
auto yplot1 = ywin1->add<Line>()->set_capacity(count);
auto swin1 = figure(plotname)->subplot(rows,cols,index<rows,cols>(1,3));
auto splot1 = swin1->add<Line>()->set_capacity(count);

auto xwin2 = figure(plotname)->subplot(rows,cols,index<rows,cols>(2,1));
auto xplot2 = xwin2->add<Line>()->set_capacity(count);
auto ywin2 = figure(plotname)->subplot(rows,cols,index<rows,cols>(2,2));
auto yplot2 = ywin2->add<Line>()->set_capacity(count);
auto swin2 = figure(plotname)->subplot(rows,cols,index<rows,cols>(2,3));
auto splot2 = swin2->add<Line>()->set_capacity(count);

auto xwin3 = figure(plotname)->subplot(rows,cols,index<rows,cols>(3,1));
auto xplot3 = xwin3->add<Line>()->set_capacity(count);
auto ywin3 = figure(plotname)->subplot(rows,cols,index<rows,cols>(3,2));
auto yplot3 = ywin3->add<Line>()->set_capacity(count);
auto swin3 = figure(plotname)->subplot(rows,cols,index<rows,cols>(3,3));
auto splot3 = swin3->add<Line>()->set_capacity(count);

auto xwin4 = figure(plotname)->subplot(rows,cols,index<rows,cols>(4,1));
auto xplot4 = xwin4->add<Line>()->set_capacity(count);
auto ywin4 = figure(plotname)->subplot(rows,cols,index<rows,cols>(4,2));
auto yplot4 = ywin4->add<Line>()->set_capacity(count);
auto swin4 = figure(plotname)->subplot(rows,cols,index<rows,cols>(4,3));
auto splot4 = swin4->add<Line>()->set_capacity(count);

void sensorResponseHandler(const U8* msg, const std::size_t) {
    //~ std::cout << "Got message" << std::endl;
    SensorMessage m = os::fromBytes<SensorMessage>(msg);
    static int t = 0; ++t;
    aplot << std::make_pair(t, (S16)le16toh(m.distance[0]));
    bplot << std::make_pair(t, (S16)le16toh(m.distance[1]));
    cplot << std::make_pair(t, (S16)le16toh(m.distance[2]));
    dplot << std::make_pair(t, (S16)le16toh(m.distance[3]));


    //~ std::cout
        //~ << std::setw(8) << (float)(S16)le16toh(m.distance[0])
        //~ << std::setw(8) << (S16)le16toh(m.distance[1])
        //~ << std::setw(8) << (S16)le16toh(m.distance[2])
        //~ << std::setw(8) << (S16)le16toh(m.distance[3])
        //~ << std::endl;
}

void irResponseHandler(const U8* msg, const std::size_t) {
    //~ std::cout << "Got message" << std::endl;
    IrCameraMessage m = os::fromBytes<IrCameraMessage>(msg);
    static int t = 0; ++t;
    xplot1 << std::make_pair(t, (U16)le16toh(m.blobs[0][0]));
    yplot1 << std::make_pair(t, (U16)le16toh(m.blobs[0][1]));
    splot1 << std::make_pair(t, (U16)le16toh(m.blobs[0][2]));

    xplot2 << std::make_pair(t, (U16)le16toh(m.blobs[1][0]));
    yplot2 << std::make_pair(t, (U16)le16toh(m.blobs[1][1]));
    splot2 << std::make_pair(t, (U16)le16toh(m.blobs[1][2]));

    xplot3 << std::make_pair(t, (U16)le16toh(m.blobs[2][0]));
    yplot3 << std::make_pair(t, (U16)le16toh(m.blobs[2][1]));
    splot3 << std::make_pair(t, (U16)le16toh(m.blobs[2][2]));

    xplot4 << std::make_pair(t, (U16)le16toh(m.blobs[3][0]));
    yplot4 << std::make_pair(t, (U16)le16toh(m.blobs[3][1]));
    splot4 << std::make_pair(t, (U16)le16toh(m.blobs[3][2]));


    std::cout
        << std::setw(8) << (S16)le16toh(m.blobs[0][0])
        << std::setw(8) << (S16)le16toh(m.blobs[0][1])
        << std::setw(8) << (S16)le16toh(m.blobs[0][2])
        << std::setw(8) << (S16)le16toh(m.blobs[1][0])
        << std::setw(8) << (S16)le16toh(m.blobs[1][1])
        << std::setw(8) << (S16)le16toh(m.blobs[1][2])
        << std::setw(8) << (S16)le16toh(m.blobs[2][0])
        << std::setw(8) << (S16)le16toh(m.blobs[2][1])
        << std::setw(8) << (S16)le16toh(m.blobs[2][2])
        << std::setw(8) << (S16)le16toh(m.blobs[3][0])
        << std::setw(8) << (S16)le16toh(m.blobs[3][1])
        << std::setw(8) << (S16)le16toh(m.blobs[3][2])
        << std::endl;
}


int main(int argc, char* argv[]){
    cpplot::glut::init(argc, argv);

    //~ typedef SerialCommunication<stm::Messages, 50, 10, B460800> Serial;
    typedef SerialCommunication<stm::Messages, 50, 10, B115200> Serial;
    Serial stm("/dev/ttyUSB0");
    stm.registerPackager<stm::Messages::sensorMessage>(sensorResponseHandler);
    stm.registerPackager<stm::Messages::irCameraMessage>(irResponseHandler);

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
