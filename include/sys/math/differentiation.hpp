#include <Eigen/Core>
#include <atomic>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <os/exceptions.hpp>
#include <iostream>

namespace sys {
    namespace math {
        using namespace Eigen;


        template<typename T, typename A, T(*FN)(const T&, const A&)>
        struct DifferentiationThread {
            static Matrix<typename T::Scalar, T::RowsAtCompileTime, T::RowsAtCompileTime> diff;
            static Matrix<typename T::Scalar, T::RowsAtCompileTime, T::RowsAtCompileTime> dx;
            static const Matrix<typename T::Scalar, T::RowsAtCompileTime, 1>* x;
            static const A* argument;
            static int id;
            static std::atomic<int> counter;
            static std::atomic<int> left;
            static std::mutex configurationGuard, leftGuard;
            static bool dying;
            static std::condition_variable resultCond;
            static std::condition_variable cond;
            int n;
            std::thread t;

            void dodediff(T(*fn)(const T&, const A&)) {
                try {
                    int lastId = 0;
                    while(!dying) {
                        std::unique_lock<std::mutex> l(configurationGuard);
                        while(id == lastId && !dying) cond.wait(l);
                        if(dying) throw os::HaltException();
                        lastId = id;
                        l.unlock();
                        diff.col(n) = (fn(*x+dx.col(n), *argument) - fn(*x-dx.col(n), *argument)) / dx(n,n) / 2;
                        --left;
                        if(left == 0) {
                            resultCond.notify_all();
                        }
                    }
                }
                catch (os::HaltException& e) {}
                catch (std::exception& e) {
                    std::cerr << e.what();
                }
            }

            DifferentiationThread()
                : n(counter++)
                , t(&DifferentiationThread<T,A,FN>::dodediff, this, FN)
            {}

            ~DifferentiationThread() {
                dying = true;
                cond.notify_all();
                t.join();
            }
        };


        template<typename T, typename A, T(*FN)(const T&, const A&)> Matrix<typename T::Scalar, T::RowsAtCompileTime, T::RowsAtCompileTime> DifferentiationThread<T,A,FN>::diff;
        template<typename T, typename A, T(*FN)(const T&, const A&)> Matrix<typename T::Scalar, T::RowsAtCompileTime, T::RowsAtCompileTime> DifferentiationThread<T,A,FN>::dx;
        template<typename T, typename A, T(*FN)(const T&, const A&)> const Matrix<typename T::Scalar, T::RowsAtCompileTime, 1>* DifferentiationThread<T,A,FN>::x;
        template<typename T, typename A, T(*FN)(const T&, const A&)> const A* DifferentiationThread<T,A,FN>::argument;
        template<typename T, typename A, T(*FN)(const T&, const A&)> std::atomic<int> DifferentiationThread<T,A,FN>::counter;
        template<typename T, typename A, T(*FN)(const T&, const A&)> std::atomic<int> DifferentiationThread<T,A,FN>::left;
        template<typename T, typename A, T(*FN)(const T&, const A&)> std::mutex DifferentiationThread<T,A,FN>::leftGuard;
        template<typename T, typename A, T(*FN)(const T&, const A&)> std::mutex DifferentiationThread<T,A,FN>::configurationGuard;
        template<typename T, typename A, T(*FN)(const T&, const A&)> std::condition_variable DifferentiationThread<T,A,FN>::resultCond;
        template<typename T, typename A, T(*FN)(const T&, const A&)> std::condition_variable DifferentiationThread<T,A,FN>::cond;
        template<typename T, typename A, T(*FN)(const T&, const A&)> int DifferentiationThread<T,A,FN>::id = 0;
        template<typename T, typename A, T(*FN)(const T&, const A&)> bool DifferentiationThread<T,A,FN>::dying = false;


        template<typename T, typename A, T(*FN)(const T&, const A&)>
        Matrix<typename T::Scalar, T::RowsAtCompileTime, T::RowsAtCompileTime> differentiate(const T& x, const T& dx, const A& arg) {
            typedef DifferentiationThread<T, A, FN> DThread;
            static DThread threads[T::RowsAtCompileTime];

            {
                std::lock_guard<std::mutex> l(DThread::configurationGuard);

                DThread::dx = dx.asDiagonal();
                DThread::x = &x;
                DThread::left = T::RowsAtCompileTime;
                DThread::argument = &arg;
                ++DThread::id;
                DThread::cond.notify_all();
            }

            std::unique_lock<std::mutex> l(DThread::leftGuard);
            while(DThread::left) DThread::resultCond.wait(l);
            return DThread::diff;
        }
    }
}
