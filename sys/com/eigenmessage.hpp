#pragma once
#ifndef SYS_COM_EIGENMESSAGE_HPP_
#define SYS_COM_EIGENMESSAGE_HPP_

#include <iostream>

namespace sys {
    struct EigenMessageBase {};
    template<typename States_>
    struct EigenMessage : public EigenMessageBase {
        typedef States_ States;
        typedef typename States::StateVector StateVector;
        StateVector value;

        std::ostream& operator<<(std::ostream& os) {
            return os << value;
        }
        EigenMessage(){}
        explicit EigenMessage(const typename States::StateVector& v) : value(v) {}
    };

    template<typename Derived>
    inline std::ostream& operator<<(std::ostream& os, const EigenMessage<Derived>& m)
    {
        return os << m.value;
    }

}

#endif
