#ifndef PATH_PLANNING_CONSTANTS
#define PATH_PLANNING_CONSTANTS

/**
 * @file constants.h
 * @brief all common constants/defines/helper-functions go in here
 * 
 * @author Georgian Besleaga, Tyler Irving Buchman, Petre Munteanu, Jacobs University Bremen, g (dot) besleaga (at) jacobs-university (dot) de, t (dot) buchman (at) jacobs-university (dot) de, p (dot) munteanu (at) jacobs-university (dot) de
 */
#include <cmath>
#define DEFAULT_MAX_SPEED 1.69
#define DEFAULT_LOOP_RATE 10

const double MAX_SPEED = DEFAULT_MAX_SPEED;
const int LOOP_RATE = DEFAULT_LOOP_RATE;

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

void checkMaxSpeed (double &v)
{
    double sign = sgn(v);
    if(abs(v) > MAX_SPEED)
    {
        v = MAX_SPEED * sign;
    }
}
#endif /* PATH_PLANNING_CONSTANTS */
