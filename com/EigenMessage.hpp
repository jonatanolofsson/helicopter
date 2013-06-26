#pragma once
#ifndef SYS_COM_EIGENMESSAGE_HPP_
#define SYS_COM_EIGENMESSAGE_HPP_

namespace sys {
    struct EigenMessageBase {};
    template<typename Contained>
    struct EigenMessage : public EigenMessageBase {
        Contained value;

        std::ostream& operator<<(std::ostream& os) {
            return os << value;
        }
        EigenMessage(){}
        explicit EigenMessage(const Contained& v) : value(v) {}
    };
        
    template<typename Derived>
    inline std::ostream& operator<<(std::ostream& os, const EigenMessage<Derived>& m)
    {
        return os << m.value;
    }

}

#endif
