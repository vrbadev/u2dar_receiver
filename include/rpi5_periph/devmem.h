#ifndef DEVMEM_H_
#define DEVMEM_H_

#include <stdint.h>

int devmem_map(uint64_t addr, uint64_t span, void** ptr);
int devmem_unmap(int devmem_fd, void* ptr, uint64_t span);

#endif
