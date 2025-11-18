#ifndef HEAP_DRIVER_H
#define HEAP_DRIVER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void heap_init(void);
void *heap_alloc(size_t size);
void heap_free(void *ptr);

/* Optional: helper to query internal info for tests/debug */
size_t heap_total_size(void);
size_t heap_block_size(void);
size_t heap_block_count(void);

#ifdef __cplusplus
}
#endif

#endif /* HEAP_DRIVER_H */
