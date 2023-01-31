#include "stepper.c"
#ifndef STEPPER_H
#define STEPPER_H

#ifdef __cplusplus
// when included in C++ file, let compiler know these are C functions
extern "C" {
#endif

void call_stepper();

#ifdef __cplusplus
}
#endif
 


#endif // stepper_H