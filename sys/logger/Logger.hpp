#pragma once
#ifndef SYS_LOGGER_HPP_
#define SYS_LOGGER_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/com/eigenmessage.hpp>
#include <Eigen/Core>
#include <fstream>
#include <type_traits>

namespace sys {
    namespace logger {
        template<typename Derived, typename Enable = void>
        struct Formatter {
            static std::ostream& format(std::ostream& os, const Derived& o) {
                return os << o;
            }
        };

        template<typename Derived>
        struct Formatter<Derived, typename std::enable_if<std::is_base_of<Eigen::DenseBase<Derived>, Derived>::value>::type> {
            static std::ostream& format(std::ostream& os, const Eigen::DenseBase<Derived>& o) {
                static Eigen::IOFormat eigenformat(Eigen::FullPrecision, Eigen::DontAlignCols, ",", ",", "", "", "", "");
                return os << o.format(eigenformat);
            }
        };

        template<typename Derived>
        struct Formatter<Derived, typename std::enable_if<std::is_base_of<EigenMessageBase, Derived>::value>::type> {
            static std::ostream& format(std::ostream& os, const Derived& o) {
                return Formatter<typename Derived::States::StateVector>::format(os, o.value);
            }
        };

        template<typename... DataTypes>
        class Logger {
            public:
                typedef Logger<DataTypes...> Self;

            private:
                void logWriteHelper() { logfile << std::endl; }

                template<typename First, typename... Other>
                void logWriteHelper(const First first, const Other... others) {
                    logfile << ",";
                    Formatter<First>::format(logfile, first);
                    logWriteHelper(others...);
                }

                template<typename First, typename... Other>
                void logWriteHelper1(const First first, const Other... others) {
                    Formatter<First>::format(logfile, first);
                    logWriteHelper(others...);
                }

                void log(const DataTypes... parameters);

            public:
                std::string filename;
                std::ofstream logfile;
                explicit Logger(const std::string filename_)
                : filename(filename_)
                , logfile(filename)
                , logDispatcher(&Self::log, this)
                {}

            private:
                os::Dispatcher<Self, DataTypes...> logDispatcher;
        };
    }
}
#endif
