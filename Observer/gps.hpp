#ifndef OS_SYS_OBSERVER_GPS_HPP_
#define OS_SYS_OBSERVER_GPS_HPP_

#include <sys/math/constants.hpp>
#include <Eigen/Core>
#include <os/com/Dispatcher.hpp>

namespace sys {
    namespace observer {
        template<typename S = Scalar>
        struct GPS {
            typedef S Scalar;
            enum state {
                x = 0,
                y = 1,
                z = 2,

                vx = 3,
                vy = 4,
                vz = 5,

                number_of_measurements = 6
            };

            enum states {
                position = x,
                velocity = vx
            };

            typedef Matrix<Scalar, number_of_measurements, 1> MeasurementVector;
            typedef Matrix<Scalar, number_of_measurements, number_of_measurements> CovarianceMatrix;
            template<typename T>
            static MeasurementVector predict(const T& filter) {
                typedef typename T::StateDescription states;
                MeasurementVector m;
                auto l = filter.retrieve_lock();
                m[x] = filter.state[states::x];
                m[y] = filter.state[states::y];
                m[z] = filter.state[states::z];

                m[vx] = filter.state[states::vx];
                m[vy] = filter.state[states::vy];
                m[vz] = filter.state[states::vz];

                return m;
            }

            template<typename T>
            static Matrix<typename T::Scalar, number_of_measurements, T::StateDescription::number_of_states> jacobian(const T&) {
                typedef typename T::StateDescription states;
                typedef Matrix<Scalar, number_of_measurements, states::number_of_states> JacobianMatrix;
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
                typedef GPS<S> sensor;
                USING_XYZ
                struct gpsOverlay {
                    float pos[3];
                    float vel[3];
                };
                if(msg.size == sizeof(gpsOverlay)) {
                    gpsOverlay* gps = (gpsOverlay*)&msg.msg;
                    Measurement<GPS> m;
                    m.z[sensor::x] = gps.pos[X];
                    m.z[sensor::y] = gps.pos[Y];
                    m.z[sensor::z] = gps.pos[Z];
                    m.z[sensor::vx] = gps.vel[X];
                    m.z[sensor::vy] = gps.vel[Y];
                    m.z[sensor::vz] = gps.vel[Z];
                    os::yield(m);
                }
            }
        };
    }
}

#endif
