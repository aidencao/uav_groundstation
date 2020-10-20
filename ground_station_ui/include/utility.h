#ifndef GS_UTILITY_H
#define GS_UTILITY_H

#include "typeinc.h"

inline void PRINT_STRING_TO_BINARY(const string& str) {
    for (auto it : str)
        printf("%02x ", (UINT8)it);
    printf("\n");
}

#endif
