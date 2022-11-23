/* Memory layout for the NXP S32K144
 * rom = 64KB (LENGTH = 0x00010000)
 * kernel = 128KB
 * user = 128KB
 * ram = 1024KB */

MEMORY
{
  rom (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00020000
  prog (rx) : ORIGIN = 0x00020000, LENGTH = 0x00020000
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 1024K
}

PAGE_SIZE = 2K;

INCLUDE ../kernel_layout.ld
