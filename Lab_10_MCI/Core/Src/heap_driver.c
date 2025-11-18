/* Core/Src/heap_driver.c
 * Minimal fixed-block heap driver — no UART in this file.
 */

#include "heap_driver.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define HEAP_START_ADDR   ((uintptr_t)0x20001000U)
#define HEAP_SIZE_BYTES   (4096U)
#define HEAP_BLOCK_SIZE   (16U)
#define HEAP_BLOCK_COUNT  (HEAP_SIZE_BYTES / HEAP_BLOCK_SIZE)

#if (HEAP_SIZE_BYTES % HEAP_BLOCK_SIZE) != 0
#error "HEAP_SIZE_BYTES must be multiple of HEAP_BLOCK_SIZE"
#endif

static uint8_t block_map[HEAP_BLOCK_COUNT];
static uint8_t * const heap_base = (uint8_t *)HEAP_START_ADDR;

void heap_init(void)
{
    memset(block_map, 0, sizeof(block_map));
}

size_t heap_total_size(void) { return HEAP_SIZE_BYTES; }
size_t heap_block_size(void) { return HEAP_BLOCK_SIZE; }
size_t heap_block_count(void) { return HEAP_BLOCK_COUNT; }

static int find_free_run(size_t need)
{
    size_t run = 0;
    for (size_t i = 0; i < HEAP_BLOCK_COUNT; ++i) {
        if (block_map[i] == 0) {
            run++;
            if (run == need) {
                return (int)(i + 1 - need);
            }
        } else {
            run = 0;
        }
    }
    return -1;
}

void *heap_alloc(size_t size)
{
    if (size == 0 || size > HEAP_SIZE_BYTES) return NULL;
    size_t need = (size + HEAP_BLOCK_SIZE - 1) / HEAP_BLOCK_SIZE;
    if (need == 0 || need > HEAP_BLOCK_COUNT) return NULL;

    int idx = find_free_run(need);
    if (idx < 0) return NULL;

    block_map[idx] = 2;
    for (size_t k = 1; k < need; ++k) block_map[idx + k] = 1;

    uintptr_t addr = HEAP_START_ADDR + (uintptr_t)idx * HEAP_BLOCK_SIZE;
    return (void *)addr;
}

void heap_free(void *ptr)
{
    if (ptr == NULL) return;

    uintptr_t uptr = (uintptr_t)ptr;
    if (uptr < HEAP_START_ADDR || uptr >= (HEAP_START_ADDR + HEAP_SIZE_BYTES)) return;

    uintptr_t offset = uptr - HEAP_START_ADDR;
    if ((offset % HEAP_BLOCK_SIZE) != 0) return;

    size_t idx = offset / HEAP_BLOCK_SIZE;
    if (idx >= HEAP_BLOCK_COUNT) return;

    if (block_map[idx] == 0) return;

    block_map[idx] = 0;
    size_t i = idx + 1;
    while (i < HEAP_BLOCK_COUNT && block_map[i] == 1) {
        block_map[i] = 0;
        i++;
    }
}
