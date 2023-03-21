/* Memory layout for the STM32F446RE
 * rom = 2MB (LENGTH = 0x02000000)
 * kernel = 256KB
 * user = 256KB
 * ram = 192KB */

MEMORY
{
  /* ivt (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00000410 */
  rom (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00040000
  prog (rx) : ORIGIN = 0x00050000, LENGTH = 0x00003BF0
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00007000
}

PAGE_SIZE = 2K;
