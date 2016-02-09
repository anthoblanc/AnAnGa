#ifndef PATHDELAY_H
#define PATHDELAY_H


class PathDelay {

public:

    PathDelay();

    float update(vector desiredPath, vector deltaLength);

private:

    struct vector prevDesiredPath;

};


#include "PathDelay.c"

#endif
