
#ifndef SFLIB_VER_H
#define SFLIB_VER_H

#include <stdio.h>

#define SFLIB_VERINFO_MAX       512
#define SFLIB_VERFUN_SUFFIX     "_print_ver"

typedef int (*SFLIB_VER_FUNC)(char*);


// name: name of lib, such as: xxx for libxxx.so
// ver : ver string, such as: "1.0.0"
// ext : extra string, such as: "g01234567" for git, or "004.xxx" for board, 
//                              or other string, 0/NULL for drop.
#define SFLIB_VER_DECLARE(name, ver, ext)       \
int lib##name##_print_ver(char *pver) {         \
    if (pver) {                                 \
        if (ext) {                              \
            snprintf(pver, SFLIB_VERINFO_MAX,   \
                "%s %s (compile time: %s %s)",  \
                ver, ext, __DATE__ ,__TIME__);  \
        } else {                                \
            snprintf(pver, SFLIB_VERINFO_MAX,   \
                "%s (compile time: %s %s)",     \
                ver, __DATE__ ,__TIME__);       \
        }                                       \
        return 0;                               \
    } else {                                    \
        return (-1);                            \
    }}


#endif  // End of SFLIB_VER_H

