//  int rand(void): returns a pseudorandom number in the range of 0 to RAND_MAX.
//  RAND_MAX: is a constant whose default value may vary between implementations but it is granted to be at least 32767.
//  void srand(int seed): initializes the random number generator.

// initializes the random number generator.
 void srand(char seed);

// returns a pseudorandom number in the range of 0 to RAND_MAX.
 int rand(void);
