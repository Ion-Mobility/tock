/* Memory layout for the STM32F446RE
 * rom = 2MB (LENGTH = 0x02000000)
 * kernel = 256KB
 * user = 256KB
 * ram = 192KB */

MEMORY
{
  ivt (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00000400
  fcf (rx)  : ORIGIN = 0x00000400, LENGTH = 0x00000010
  rom (rx)  : ORIGIN = 0x00000410, LENGTH = 0x00004000
  prog (rx) : ORIGIN = 0x00004000, LENGTH = 0x00003BF0
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00007000
}

SECTIONS {
  .vtor  : ALIGN(4)
    {
        KEEP (*(.vtor))
    } > ivt
    
}

SECTIONS {
  .flashconfig  : ALIGN(4)
    {
        KEEP (*(.flashconfig))
    } > fcf
    
}

PAGE_SIZE = 2K;
