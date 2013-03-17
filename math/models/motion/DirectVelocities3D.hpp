#ifndef SYS_MATH_MODELS_DIRECTVELOCITIES3D_HPP_
#define SYS_MATH_MODELS_DIRECTVELOCITIES3D_HPP_

#include <Eigen/Core>
#include <sys/types.hpp>
#include <sys/math/constants.hpp>
#include <sys/math/statedifferentiation.hpp>
#include <cmath>

namespace sys {
    namespace models {
        namespace motion {
            using namespace Eigen;
            struct DirectVelocities3D {
                static const char CD = 'D';

                template<typename T, typename D>
                static typename T::States diffStates(const T& filter, const D& dx) {
                    T positive(filter);
                    T negative(filter);
                    positive.state += dx;
                    negative.state -= dx;
                    return predict<T>(positive, 1e-2) - predict<T>(negative, 1e-2);
                }
                template<typename T, typename D>
                static typename T::States diffControl(const T& filter, const D& du) {
                    T positive(filter);
                    T negative(filter);
                    positive.controls += du;
                    negative.controls -= du;
                    return predict<T>(positive, 1e-2) - predict<T>(negative, 1e-2);
                }

                template<typename T>
                static typename T::States predict(const T& filter, const typename T::Scalar dT) {
                    typedef typename T::Scalar Scalar;
                    typedef typename T::States States;
                    typedef typename T::Controls Controls;
                    typedef typename T::Model::ControlDescription controls;
                    typedef typename T::Model::StateDescription states;
                    auto& x = filter.state;
                    auto& u = filter.controls;
                    //~ USING_XYZ
                    States xnext(x);

                    xnext.template segment<3>(states::position) += u.template segment<3>(controls::velocity) * dT;

                    //~ // Note that this skew-symmetric matrix is diagonally shifted upwards-left compared
                    //~ // to many representations, due to the storage model used by Eigen for quaternions: qx, qy, qz, qw
                    //~ Matrix<Scalar, 4, 4> S;
                    //~ S << QUATERNION_ROTATION_FROM_ROTVEL(x(states::omega[X]), x(states::omega[Y]), x(states::omega[Z]));
//~
                    //~ const Scalar wnorm     = x.template segment<3>(states::rotational_velocity).norm();
                    //~ const Scalar wnormT2   = wnorm * filter.dT / 2;
//~
                    //~ if(wnorm > math::EPSILON) {
                        //~ xnext.template segment<4>(states::quaternion) =
                            //~ (std::cos(wnormT2) * x.template segment<4>(states::quaternion)
                                //~ - (std::sin(wnormT2) / wnorm) * S * x.template segment<4>(states::quaternion)
                            //~ ).normalized();
                    //~ }
//~
                    //~ xnext.template segment<3>(states::rotational_velocity) = u.template segment<3>(controls::rotational_velocity);

                    return xnext;
                }

                template<typename T>
                static Matrix<typename T::States::Scalar, T::States::RowsAtCompileTime, T::States::RowsAtCompileTime>
                systemJacobian(const T& filter) {
                    static const typename T::States dx = T::States::Constant(1e-2);
                    return math::template differentiateStates<T, typename T::States, &DirectVelocities3D::diffStates<T>>(filter, dx);
                }

                template<typename T>
                static Matrix<typename T::Controls::Scalar, T::States::RowsAtCompileTime, T::Controls::RowsAtCompileTime>
                controlJacobian(const T& filter) {
                    static const typename T::Controls du = T::Controls::Constant(1e-2);
                    return math::template differentiateStates<T, typename T::Controls, &DirectVelocities3D::diffControl<T>>(filter, du);
                }

                template<typename T>
                static Matrix<typename T::States::Scalar, T::States::RowsAtCompileTime, T::States::RowsAtCompileTime> covariance() {
                    typedef Matrix<typename T::States::Scalar, T::States::RowsAtCompileTime, T::States::RowsAtCompileTime> RetType;
                    return RetType::Identity();
                }
            };
        }
    }
}

#endif
