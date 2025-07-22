//
// Created by shawn on 25-7-17.
//

#include <stdlib.h>

void* operator new(size_t size) { return malloc(size); }
void* operator new[](size_t size) { return malloc(size); }

void operator delete(void* ptr, size_t) { free(ptr); }
void operator delete[](void* ptr, size_t) { free(ptr); }
void operator delete(void* ptr) { free(ptr); }
void operator delete[](void* ptr) { free(ptr); }
