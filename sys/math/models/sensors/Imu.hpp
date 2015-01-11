#pragma once

#include <sys/math/constants.hpp>
#include <sys/math/filtering/GaussianFilter.hpp>
#include <Eigen/Core>
#include <os/com/Dispatcher.hpp>
#include <sys/types.hpp>
#include <sys/math/statistics.hpp>

namespace sys {
    namespace math {
        namespace models {
            using namespace Eigen;

            struct ImuStraightOff {
                typedef AW_3D States;
                typedef States states;
                typedef States::StateVector Result;
                using StateVector = States::StateVector;

                template<typename ExternalStates>
                StateVector predict(ExternalStates& x, Scalar = settings::dT) {
                    typedef ExternalStates extstates;
                    Result m;
                    m[states::ax] = x[extstates::ax];
                    m[states::ay] = x[extstates::ay];
                    m[states::az] = x[extstates::az];

                    m[states::wx] = x[extstates::wx];
                    m[states::wy] = x[extstates::wy];
                    m[states::wz] = x[extstates::wz];

                    return m;
                }
            };

            template<typename MotionModel_ = ImuStraightOff>
            struct Imu {
                typedef MotionModel_ MotionModel;
                typedef Imu Self;
                typedef AW_3D States;
                typedef States::StateVector Result;
                static const int nofStates = States::nofStates;
                static const int frequency = 100;

                template<typename ExternalStates>
                static Result measurement(const typename ExternalStates::StateVector& x) {
                    Result r = States::template translateFrom<typename MotionModel::States>(MotionModel::template predict<ExternalStates>(x));
                    return r;
                }

                static internal::Covariance<nofStates> covariance() {
                    typedef Matrix<Scalar, nofStates, nofStates> RetType;
                    return RetType::Identity();
                }

                template<typename ExternalStates>
                static Matrix<Scalar, nofStates, ExternalStates::nofStates>
                observationMatrix(const typename ExternalStates::StateVector& x) {
                    auto J = math::differentiate<MotionModel, ExternalStates, ExternalStates>(x);
                    return States::translateColumns<typename MotionModel::States, ExternalStates>(J);
                }

                static Result noise() {
                    return math::normalSample(covariance());
                }
            };
        }
    }
}

