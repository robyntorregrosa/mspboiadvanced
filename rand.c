#include <rand.h>

//  int rand(void): returns a pseudorandom number in the range of 0 to RAND_MAX.
//  RAND_MAX: is a constant whose default value may vary between implementations but it is granted to be at least 32767.
//  void srand(int seed): initializes the random number generator.

#define RAND_MAX 31
#define BIT5 0x20;
#define BIT3 0x8;

int lfsr;
int *lfsrPtr = &lfsr;

 void srand(char seed) {
     if (seed == 0) {
         seed = 1;
     }
     lfsr = seed;   // seed with nonzero value
 }

 int rand()
 {      // thanks wikipedia: https://en.wikipedia.org/wiki/Linear-feedback_shift_register
     lfsr ^= lfsr >> 7;
     lfsr ^= lfsr << 9;
     lfsr ^= lfsr >> 13;
     return lfsr;
 }
