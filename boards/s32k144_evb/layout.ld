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
  prog (rx) : ORIGIN = 0x00020000, LENGTH = 0x00060000
  ram (rwx) : ORIGIN = 0x1FFF8000 , LENGTH = 60KB
}

PAGE_SIZE = 2K;

INCLUDE ./kernel_layout.ld
