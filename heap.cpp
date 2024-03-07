/*
 * heap.cpp
 *
 *  Created on: 12 jun. 2020
 *      Author: jcabe
 */


#include "ch.hpp"
#include "hal.h"
using namespace chibios_rt;


void* operator new( size_t size ) { return chHeapAlloc( 0x0, size ); }
void* operator new[]( size_t size ) { return chHeapAlloc( 0x0, size ); }
void operator delete( void *p ) { chHeapFree( p ); }
void operator delete[]( void *p ) { chHeapFree( p ); }

