#ifndef TYPEINC_H
#define TYPEINC_H

//type
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
#include <ros/types.h>


//sys
#include <sys/time.h>
#include <unistd.h>
#include <malloc.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <termios.h>

//lib
#include <stdlib.h>
#include <cstdlib>
#include <math.h>


//io
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <fstream>


//stl
#include <string.h>
#include <queue>
#include <vector>
using std::vector;
using std::queue;
using std::string;
using std::cin;
using std::cout;
using std::endl;


//costom
typedef uint8_t UINT8;
typedef uint16_t UINT16;
typedef uint32_t UINT32;
typedef uint64_t UINT64;

typedef int8_t INT8;
typedef int16_t INT16;
typedef int32_t INT32;
typedef int64_t INT64;

#endif // TYPEINC_H
