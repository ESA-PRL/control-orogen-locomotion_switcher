#ifndef locomotion_switcher_TYPES_HPP
#define locomotion_switcher_TYPES_HPP

namespace locomotion_switcher
{
    enum LocomotionMode
    {
        DONT_CARE=-1,
        DRIVING=0,
        WHEEL_WALKING=1,
        CRABBING=2,
        NUM_MODES=3
    };
}

#endif // locomotion_switcher_TYPES_HPP
