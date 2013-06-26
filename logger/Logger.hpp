#pragma once
#ifndef SYS_LOGGER_HPP_
#define SYS_LOGGER_HPP_

#include <os/com/Dispatcher.hpp>
#include <sys/com/EigenMessage.hpp>
#include <Eigen/Core>
#include <fstream>
#include <type_traits>

namespace sys {
    namespace logger {
        template<typename Derived>
        std::ostream& operator<<(std::ostream& os, const Eigen::EigenBase<Derived>& o)
        {
            static Eigen::IOFormat eigenformat(Eigen::FullPrecision, Eigen::DontAlignCols, ",", ",", "", "", "", "");
            return os << o.format(eigenformat);
        }
        
        template<typename... DataTypes>
        class Logger {
            public:
                typedef Logger<DataTypes...> Self;

            private:
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
