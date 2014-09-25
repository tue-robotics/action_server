#include "action_server/uuid.h"

#include <stdlib.h>

// Returns the number of CPU cycles since powered on
// See: http://stackoverflow.com/questions/7617587/is-there-an-alternative-to-using-time-to-seed-a-random-number-generation
unsigned long long rdtsc()
{
    unsigned int lo,hi;
    __asm__ __volatile__ ("rdtsc" : "=a" (lo), "=d" (hi));
    return ((unsigned long long)hi << 32) | lo;
}

namespace act
{

// ----------------------------------------------------------------------------------------------------

UUID UUID::generate()
{
    // We need to seed the random generator, but only the first time generate() is called. Therefore,
    // create a static wrapper class (only constructed once) with the seed call in the constructor
    // body.
    static class Once { public: Once() { srand(rdtsc()); } } Once_;

    static const char alphanum[] =
        "0123456789"
        "abcdefghijklmnopqrstuvwxyz"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ";

    UUID ID;
    for (int i = 0; i < 20; ++i) {
//        int n = rand() / (RAND_MAX / (sizeof(alphanum) - 1) + 1);
        int n = rand() % (sizeof(alphanum) - 1);
        ID.id_ += alphanum[n];
    }

    return ID;
}

}
