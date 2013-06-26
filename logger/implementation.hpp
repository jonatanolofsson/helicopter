#pragma once
#ifndef SYS_LOGGER_IMPLEMENTATION_HPP_
#define SYS_LOGGER_IMPLEMENTATION_HPP_

#include <os/type_traits.hpp>
#include <sys/Logger.hpp>

namespace sys {
    namespace logger {
        template<typename... DataTypes>
        void Logger<DataTypes...>::log(const DataTypes... parameters)
        {
            os::eval((logfile << parameters <<",")...);
            logfile << std::endl;
        }
    }
}

#endif
