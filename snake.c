/*
 * snake.c
 *
 *  Created on: Apr 28, 2019
 *      Author: Robyn
 */

// C program for array implementation of Snake
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

// a FIFO Snake
unsigned char SNAKE[3] = {222, 222, 0};

struct Snake
{
    int head, tail, size;
    unsigned capacity;
    int* array;
};

// function to create a Snake of given capacity.
// It initializes size of Snake as 0
struct Snake* createSnake(unsigned capacity)
{
    struct Snake* Snake = (struct Snake*) malloc(sizeof(struct Snake));
    Snake->capacity = capacity;
    Snake->head = Snake->size = 0;
    Snake->tail = capacity - 1;  // This is important, see the enSnake
    Snake->array = (int*) malloc(Snake->capacity * sizeof(int));
    return Snake;
}

// Snake is full when size becomes equal to the capacity
int isFull(struct Snake* Snake)
{  return (Snake->size == Snake->capacity);  }

// Snake is empty when size is 0
int isEmpty(struct Snake* Snake)
{  return (Snake->size == 0); }

// Function to add an item to the Snake.
// It changes tail and size
void enSnake(struct Snake* Snake, int item)
{
    if (isFull(Snake))
        return;
    Snake->tail = (Snake->tail + 1)%Snake->capacity;
    Snake->array[Snake->tail] = item;
    Snake->size = Snake->size + 1;
    printf("%d enSnaked to Snake\n", item);
}

// Function to remove an item from Snake.
// It changes head and size
int deSnake(struct Snake* Snake)
{
    if (isEmpty(Snake))
        return INT_MIN;
    int item = Snake->array[Snake->head];
    Snake->head = (Snake->head + 1)%Snake->capacity;
    Snake->size = Snake->size - 1;
    return item;
}

// Function to get head of Snake
int head(struct Snake* Snake)
{
    if (isEmpty(Snake))
        return INT_MIN;
    return Snake->array[Snake->head];
}

// Function to get tail of Snake
int tail(struct Snake* Snake)
{
    if (isEmpty(Snake))
        return INT_MIN;
    return Snake->array[Snake->tail];
}

// Function to get pointer to Snake head
int *headPtr(struct Snake* Snake)
{
    return &Snake->array[Snake->head];
}

//void getNextPt(unsigned char direction, int head, *unsigned char nextPtPtr){
//
//    *nextPtx++ = headx + direction;
//    *nextPty = heady + direction;
//
//}
