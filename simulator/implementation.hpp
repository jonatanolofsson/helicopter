#pragma once
#ifndef SYS_SIMULATOR_IMPLEMENTATION_HPP_
#define SYS_SIMULATOR_IMPLEMENTATION_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/com/Maple.hpp>
#include <sys/com/Stm.hpp>
#include <sys/simulator/API.hpp>
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

            auto mImu = sensors::Imu::measurement(filter.state);
            os::yield(maple::SensorMessage{{
                (S16)(accelerometerScaling * mImu[0]),
                (S16)(accelerometerScaling * mImu[1]),
                (S16)(accelerometerScaling * mImu[2]),

                (S16)(gyroscopeScaling * mImu[3]),
                (S16)(gyroscopeScaling * mImu[4]),
                (S16)(gyroscopeScaling * mImu[5])
            },0,{0},{0},{0},0,0});

            sys::math::GaussianMeasurement<PfSensor, true> m;
            m.sensor = &pfSensor;

            auto z = m.measurement(filter.state);
            os::yield(stm::SensorMessage{{
                (U16)(distanceScaling * z[0]),
                (U16)(distanceScaling * z[1]),
                (U16)(distanceScaling * z[2])
            }});
        }
    }
}

#endif
