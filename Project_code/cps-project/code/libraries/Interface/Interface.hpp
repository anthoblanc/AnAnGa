#ifndef INTERFACE_HPP
#define INTERFACE_HPP



class Interface {

public:

    Interface(const AP_HAL::HAL& hal);

    void update(char inputString);

private:
    const AP_HAL::HAL& m_rHAL;

};


#include "Interface.cpp"

#endif
