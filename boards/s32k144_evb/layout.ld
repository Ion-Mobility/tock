/* Memory layout for the NXP S32K144
 * rom =
 * kernel = 
 * user = 
 * ram =  */

MEMORY
{
  ivt (rx)  : ORIGIN = 0x00000000, LENGTH = 0x400
  fcf (ro)  : ORIGIN = 0x00000400, LENGTH = 0x10
  rom (rx)  : ORIGIN = 0x00000410, LENGTH = 0x00020000
  prog (rx) : ORIGIN = 0x00020000, LENGTH = 0x00070000
  ram (rwx) : ORIGIN = 0x1ffffc00 , LENGTH = 64KB
}

PAGE_SIZE = 2K;

INCLUDE ../kernel_layout.ld
