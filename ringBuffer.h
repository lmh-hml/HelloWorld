#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <ros/ros.h>
#include <iostream>

struct Int16_RingBuffer
{
    Int16_RingBuffer()
    {
        oldest = 0;
        newest = 0;
    }

    const int size =10 ;
    int16_t array[10];
    int oldest;
    int newest;

    void insert( int16_t to_insert)
    {
        array[newest] = to_insert;
        newest++;

    }

};


#endif // RINGBUFFER_H
