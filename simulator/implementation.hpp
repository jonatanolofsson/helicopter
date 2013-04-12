#pragma once
#ifndef SYS_SIMULATOR_IMPLEMENTATION_HPP_
#define SYS_SIMULATOR_IMPLEMENTATION_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/com/Maple.hpp>
#include <sys/com/Stm.hpp>
#include <sys/simulator/API.hpp>
#include <sys/sensorhub/API.hpp>
#include <sys/settings.hpp>
#include <sys/math/models/Map.hpp>
#include <sys/math/models.hpp>

namespace sys {
    namespace simulator {

        typedef sys::math::models::UltrasoundInMap<ModelDescription, 4, sys::math::Map<14>> PfSensor;
        extern PfSensor pfSensor;

        template<typename MotionModel_, typename Filter_>
        Simulator<MotionModel_, Filter_>::Simulator()
        : controlDispatcher(&Self::simulate, this)
        {
            MotionModel::ModelDescription::StateDescription::initialize(filter);
        }

        template<typename MotionModel_, typename Filter_>
        void Simulator<MotionModel_, Filter_>::simulate(const Controls u) {
            filter.state = MotionModel::predict(filter.state, u, settings::dT);// + filter.noise();

            /* Measurements for the kalman filter */
            auto mImu = sensors::Imu::measurement(filter.state) + sensors::Imu::noise();
            os::yield(maple::SensorMessage{{
                (S16)(accelerometerScaling * mImu[0]), 0, 0,
                0, 0, (S16)(gyroscopeScaling * mImu[1])
            },0,{0},{0},{0},0,0});

            sys::sensorhub::sensors::Mouse m;
            m.z = sensors::Mouse::measurement(filter.state) + sensors::Mouse::noise();
            os::yield(m);

            /* Measurements for the particlefilter */
            static int pN = 0;
            pN = (pN + 1) % 5;
            if(pN == 0) {
                sys::math::GaussianMeasurement<PfSensor, true> m;
                m.sensor = &pfSensor;
                auto z = m.measurement(filter.state) + PfSensor::noise();
                os::yield(stm::SensorMessage{{
                    (U16)(distanceScaling * z[0]),
                    (U16)(distanceScaling * z[1]),
                    (U16)(distanceScaling * z[2])
                }});
            }
        }
    }
}

#endif
