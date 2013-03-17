#ifndef SYS_MATH_MODELS_SENSORS_GPS_HPP_
#define SYS_MATH_MODELS_SENSORS_GPS_HPP_

#include <sys/math/constants.hpp>
#include <sys/math/filtering/GaussianFilter.hpp>
#include <Eigen/Core>
#include <os/com/Dispatcher.hpp>
#include <sys/types.hpp>

namespace sys {
    namespace models {
        namespace sensors {
            using namespace Eigen;
            struct Gps {
                typedef Gps Self;
                typedef sys::Scalar Scalar;
                enum state {
                    x = 0,
                    y = 1,
                    z = 2,

                    vx = 3,
                    vy = 4,
                    vz = 5,

                    nofMeasurements = 6
                };

                enum states {
                    position = x,
                    velocity = vx
                };

                typedef Matrix<Scalar, nofMeasurements, 1> MeasurementVector;
                typedef Matrix<Scalar, nofMeasurements, nofMeasurements> CovarianceMatrix;
                template<typename T>
                static MeasurementVector measurement(const T& filter) {
                    typedef typename T::Model::StateDescription states;
                    MeasurementVector m;
                    m[x] = filter.state[states::x];
                    m[y] = filter.state[states::y];
                    m[z] = filter.state[states::z];

                    m[vx] = filter.state[states::vx];
                    m[vy] = filter.state[states::vy];
                    m[vz] = filter.state[states::vz];

                    return m;
                }

                template<typename T>
                static Matrix<typename T::Scalar, nofMeasurements, T::Model::nofStates>
                jacobian(const T&) {
                    typedef typename T::Model::StateDescription states;
                    typedef Matrix<Scalar, nofMeasurements, states::nofStates> JacobianMatrix;
                    JacobianMatrix J;
                    J.setZero();

                    J(x,states::x) = 1;
                    J(y,states::y) = 1;
                    J(z,states::z) = 1;

                    J(vx,states::vx) = 1;
                    J(vy,states::vy) = 1;
                    J(vz,states::vz) = 1;

                    return J;
                }


                template<typename PostOffice>
                static void packager(const typename PostOffice::Message& msg) {
                    USING_XYZ
                    struct gpsOverlay {
                        float pos[3];
                        float vel[3];
                    };
                    if(msg.size == sizeof(gpsOverlay)) {
                        gpsOverlay* gps = (gpsOverlay*)&msg.msg;
                        math::GaussianMeasurement<Self> m;
                        m.z[x] = gps.pos[X];
                        m.z[y] = gps.pos[Y];
                        m.z[z] = gps.pos[Z];
                        m.z[vx] = gps.vel[X];
                        m.z[vy] = gps.vel[Y];
                        m.z[vz] = gps.vel[Z];
                        os::yield(m);
                    }
                }
            };
        }
    }
}

#endif
