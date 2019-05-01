#include <rand.h>

//  int rand(void): returns a pseudorandom number in the range of 0 to RAND_MAX.
//  RAND_MAX: is a constant whose default value may vary between implementations but it is granted to be at least 32767.
//  void srand(int seed): initializes the random number generator.

#define RAND_MAX 31
#define BIT5 0x20;
#define BIT3 0x8;

unsigned char lfsr;
unsigned char *lfsrPtr = &lfsr;

 void srand(char seed) {
     *lfsrPtr = seed;
 }

int rand(void) {
     int newbit;
     // taps 3, 5
     newbit = ((*lfsrPtr >> 0) ^ (*lfsrPtr >> 2)) & 1;
     *lfsrPtr = (*lfsrPtr >> 1) | (newbit << 4);    // shift buffer values, mod by maximum
     return (*lfsrPtr & 0x3) + 1;         // return value 1-4
 }
