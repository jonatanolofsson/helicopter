#pragma once
#ifndef SYS_MATH_MODELS_VELOCITY_XQ_3D_HPP_
#define SYS_MATH_MODELS_VELOCITY_XQ_3D_HPP_

#include <Eigen/Core>
#include <sys/math/models/motion.hpp>
#include <sys/types.hpp>
#include <sys/math/states.hpp>
#include <sys/math/constants.hpp>
#include <sys/math/algorithm.hpp>
#include <cmath>

#include <iostream>

namespace sys {
    namespace math {
        namespace models {
            using namespace Eigen;
            template<typename States_ = XQ_3D>
            struct Velocity_XQ_3D : public Model<Velocity_XQ_3D<States_>, States_> {
                typedef Velocity_XQ_3D<States_> Self;
                typedef Model<Self, States_> Base;
                using States = typename Base::States;
                using StateVector = typename Base::StateVector;

                template<typename ExternalStates>
                static StateVector predict(const typename ExternalStates::StateVector& x, const Scalar dT = settings::dT) {
                    //LOG_EVENT(typeid(Self).name(), 50, "Predicting for x=" << x.transpose());
                    //LOG_EVENT(typeid(Self).name(), 50, "dT=" << dT);
                    typedef ExternalStates extstates;
                    StateVector xnext = States::template translate<ExternalStates>(x);

                    xnext.template segment<3>(States::position) += x.template segment<3>(extstates::velocity) * dT;

                    // Note that this skew-symmetric matrix is diagonally shifted upwards-left compared
                    // to many representations, due to the storage model used by Eigen for quaternions: qx, qy, qz, qw
                    Matrix<Scalar, 4, 4> S;
                    S << QUATERNION_ROTATION_FROM_ROTVEL(x(extstates::wx), x(extstates::wy), x(extstates::wz));

                    const Scalar wnorm     = x.template segment<3>(extstates::omega).norm();

                    if(wnorm > math::constants::EPSILON) {
                        const Scalar wnormT2   = wnorm * dT / 2;
                        xnext.template segment<4>(States::quaternion) =
                            (std::cos(wnormT2) * x.template segment<4>(extstates::quaternion)
                                - (std::sin(wnormT2) / wnorm) * S * x.template segment<4>(extstates::quaternion)
                            ).normalized();
                    } else {
                        xnext.template segment<4>(States::quaternion) = x.template segment<4>(extstates::quaternion);
                    }
                    //LOG_EVENT(typeid(Self).name(), 50, "--> xnext = " << xnext.transpose());

                    return xnext;
                }
            };
        }
    }
}

#endif
