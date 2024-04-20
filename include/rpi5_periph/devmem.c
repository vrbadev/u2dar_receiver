#include <unistd.h>
#include <fcntl.h> 
#include <sys/mman.h> 

#include "devmem.h"


int devmem_map(uint64_t addr, uint64_t span, void** ptr)
{
    int devmem_fd;

    if ((devmem_fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) {
        return -1;
    }

    int64_t pagesize = sysconf(_SC_PAGESIZE);
    uint64_t aligned_offset = addr & -pagesize;
    uint64_t span_add = addr - aligned_offset; 
    void* map = mmap(NULL, span + span_add, PROT_READ | PROT_WRITE, MAP_SHARED, devmem_fd, aligned_offset);
    if (map == MAP_FAILED) {
        close(devmem_fd);
        return -2;
    }
    *ptr = (void*) (map + span_add); 

    return devmem_fd;
}

int devmem_unmap(int devmem_fd, void* ptr, uint64_t span)
{
    int64_t pagesize = sysconf(_SC_PAGESIZE);
    uint64_t aligned = ((uint64_t) ptr) & -pagesize;
    uint64_t span_add = ((uint64_t) ptr) - aligned; 
    if (munmap((void*) (aligned), span + span_add) != 0) {
		return -1;
    }
	close(devmem_fd);
    return 0;
}
