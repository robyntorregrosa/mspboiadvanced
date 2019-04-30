/*
 * snake.h
 *
 *  Created on: Apr 28, 2019
 *      Author: Robyn
 */

#ifndef SNAKE_H_
#define SNAKE_H_

#include <rand.h>

// initialize a snake object
struct Snake* createSnake(unsigned capacity);

// Function to remove an item from Snake.
// It changes head and size
int deSnake(struct Snake* Snake);

// Function to get head of Snake
int head(struct Snake* Snake);

// Function to get tail of Snake
int tail(struct Snake* Snake);


#endif /* SNAKE_H_ */
