#ifndef INTERFACE_HPP
#define INTERFACE_HPP

#include <AP_HAL.h>


class Interface {

public:

    Interface(AP_HAL::HAL& hal);

    void update(char inputString);

};

#endif
