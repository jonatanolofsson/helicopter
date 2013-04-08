#include <cmath>
#include <algorithm>
#include <sys/math/models.hpp>
#include <sys/math/models/Map.hpp>
#include <sys/math/filtering.hpp>
#include <sys/math/states.hpp>
#include <sys/math/statistics.hpp>
#include <Eigen/Core>

#include <cpplot/cpplot.hpp>
using namespace cpplot;

using namespace Eigen;
using namespace sys;
static const int nofParticles = 5000;
typedef sys::math::models::S2DPose states;
typedef sys::math::models::CVW controls;
typedef sys::math::models::Description<states, controls> ModelDescription;
typedef sys::math::models::CoordinatedTurn2DPose<ModelDescription> MotionModel;
typedef sys::math::ParticleFilter<ModelDescription, nofParticles> Filter;
typedef sys::math::PF Algorithm;

template<int rows, int cols>
int index(int row, int col) { return row + (col-1)*rows; }


static const int count = 30*10;
#define rows 1
#define cols 1
#define plotname "PF"


int main(int argc, char* argv[]){
    cpplot::glut::init(argc, argv);
    auto mapwin = figure(plotname)->subplot(rows,cols,index<rows,cols>(1,1))->title("Map");
    //~ double X[14][2] = {
        //~ {0.00, 2.40}, {0.00, 2.40}, {2.40, 0.00}, {2.40, 0.00},
        //~ {0.69, 1.51}, {0.69, 0.46}, {1.17, 0.00}, {1.17, 1.37}, {1.94, 1.37},
        //~ {1.17, 1.94}, {0.46, 1.51}, {1.65, 1.37}, {0.00, 1.01}, {1.17, 0.89} };
    //~ double Y[14][2] = {
        //~ {0.00, 2.40}, {2.40, 2.40}, {2.40, 0.00}, {0.00, 0.00},
        //~ {0.69, 2.40}, {0.69, 1.01}, {1.17, 0.46}, {1.17, 1.94}, {1.94, 1.94},
        //~ {1.94, 1.94}, {0.69, 1.51}, {1.94, 1.37}, {0.69, 1.01}, {2.40, 0.89} };

    double W[14][2][2] = {
        // border
        {{0.00, 0.00}, {0.00, 2.40}},
        {{0.00, 2.40}, {2.40, 2.40}},
        {{2.40, 2.40}, {2.40, 0.00}},
        {{2.40, 0.00}, {0.00, 0.00}},

        // vertical walls
        {{0.69, 1.51}, {0.69, 2.40}},
        {{0.69, 0.46}, {0.69, 1.01}},
        {{1.17, 0.00}, {1.17, 0.46}},
        {{1.17, 1.37}, {1.17, 1.94}},
        {{1.94, 1.37}, {1.94, 1.94}},

        // horizontal walls
        {{1.17, 1.94}, {1.94, 1.94}},
        {{0.46, 1.51}, {0.69, 1.51}},
        {{1.65, 1.37}, {1.94, 1.37}},
        {{0.00, 1.01}, {0.69, 1.01}},
        {{1.17, 0.89}, {2.40, 0.89}}
    };
    line_t mapwalls[14];
    for(int i = 0; i < 14; ++i) {
        std::vector<double> wX, wY;
        wX.push_back(W[i][0][0]);
        wY.push_back(W[i][0][1]);
        wX.push_back(W[i][1][0]);
        wY.push_back(W[i][1][1]);
        mapwalls[i] = mapwin->add<Line>()->line(wX, wY)->set("LineWidth", 1.0)->set("k");
    }

    //~ auto wallplot = mapwin->add<Patch>()->patch(wX,wY,wZ);

    Filter filter;
    ModelDescription::Controls u;

    states::initialize(filter);
    u.setZero();
    u[1] = 5;
    for(auto& p : filter.particles) {
        p.state.setZero();
        p.weight = 1.0/nofParticles;
    }
    filter.maxWeight = 1.0/nofParticles;

    typedef sys::math::models::UltrasoundInMap<ModelDescription, 4, sys::math::Map<14>> Sensor;

    Sensor sensor{{
        {Sensor::SensorType::Position(0.1, 0), 0},
        {Sensor::SensorType::Position(0.0, 0.1), M_PI_2},
        {Sensor::SensorType::Position(-0.1, 0), M_PI},
        {Sensor::SensorType::Position(0.0, -0.1), -M_PI_2}
    },
    {{
        // border
        {{0.00, 0.00}, {0.00, 2.40}},
        {{0.00, 2.40}, {2.40, 2.40}},
        {{2.40, 2.40}, {2.40, 0.00}},
        {{2.40, 0.00}, {0.00, 0.00}},

        // vertical walls
        {{0.69, 1.51}, {0.69, 2.40}},
        {{0.69, 0.46}, {0.69, 1.01}},
        {{1.17, 0.00}, {1.17, 0.46}},
        {{1.17, 1.37}, {1.17, 1.94}},
        {{1.94, 1.37}, {1.94, 1.94}},

        // horizontal walls
        {{1.17, 1.94}, {1.94, 1.94}},
        {{0.46, 1.51}, {0.69, 1.51}},
        {{1.65, 1.37}, {1.94, 1.37}},
        {{0.00, 1.01}, {0.69, 1.01}},
        {{1.17, 0.89}, {2.40, 0.89}}
    }}};

    //~ filter.state << 0.4, 0.7, 0.0;
    filter.state << 1.5, 1.6, 0.0;
    ModelDescription::States trueState = filter.state;
    line_t distPlot;
    line_t truth = mapwin->add<Line>()->set("LineStyle", "none")->set("r")->set("Marker", "d");
    line_t guess = mapwin->add<Line>()->set("LineStyle", "none")->set("g")->set("Marker", "o");

    for(auto& p : filter.particles) {
        p.state << sys::math::randU()*2.2 + 0.1, sys::math::randU()*2.2 + 0.1, sys::math::randU()*2*M_PI;
        p.weight = 1.0/nofParticles;
    }
    distPlot = mapwin->add<Line>()->set("LineStyle", "none")->set("b")->set("Marker", "x");

    sys::math::GaussianMeasurement<Sensor, true> m;
    m.sensor = &sensor;
    m.R.setIdentity();
    m.R *= 0.1;


    int i = 0;
    while(true) {
        cpplot::dvec pX, pY, gX, gY, tX, tY;
        for(auto& p : filter.particles) {
            pX.push_back(p.state[0]);
            pY.push_back(p.state[1]);
        }
        distPlot->line(pX, pY);

        trueState = MotionModel::predict(trueState, u, 1e-2);
        tX.push_back(trueState[states::x]);
        tY.push_back(trueState[states::y]);
        truth->line(tX, tY);

        m.z = m.measurement(trueState);
        gX.push_back(filter.state[states::x]);
        gY.push_back(filter.state[states::y]);
        guess->line(gX, gY);

        usleep(100000);

        Algorithm::update<MotionModel>(filter, u, m, 1e-2);

        filter.state(states::th) = sys::math::toPi(filter.state(states::th));
        trueState(states::th) = sys::math::toPi(trueState(states::th));
        std::cout << "[" << i++ << "] Truth and best guess: " << trueState.transpose() << " == " << filter.state.transpose() << std::endl;
    }
}
