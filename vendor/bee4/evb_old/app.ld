/* Entry Point */
ENTRY(Reset_Handler)

MEMORY
{
NS_APP (r) : ORIGIN = 0x0405F000, LENGTH = 0x00100000
RAM_VECTOR_SECTION (rwx) : ORIGIN = 0x14EC00, LENGTH = 0x2EC
RAM_VECTOR_WRAPPER (rwx) : ORIGIN = 0x100C00, LENGTH = 0x240
ER_IRAM_NS (rwx) : ORIGIN = (0x100C00 + 0x240), LENGTH = (170 * 1024 - 0x240) 
EXT_DATA_RAM (rwx) : ORIGIN = 0x200000, LENGTH = 0x4000
APP_TRACE (r) : ORIGIN = 0x18100000, LENGTH = 0x80000
}

/* _estack = 0x13E000; */
/* Define output sections */
SECTIONS
{
    FLASH_HEADER  :
    {
        KEEP(* (.app.flash.header))
    } > NS_APP
    __app_flash_load_ad__ = LOADADDR(FLASH_HEADER);

    FLASH_HEADER_EXT :
    {
       KEEP(* (.app.flash.header_ext))
    } > NS_APP

    FLASH_TEXT :
    {
        __flash_text_start__ = .;
        KEEP(* (.image_entry))
        *(EXCLUDE_FILE (*libbee4-internal.a:*.obj *libopenthread-bee4.a:*.obj *libopenthread-ftd.a:*mac_frame.obj *libopenthread-ftd.a:*crypto_platform.obj *libopenthread-ftd.a:*aes_ccm.obj *libopenthread-ftd.a:*aes_ecb.obj *libmbedcrypto.a:*aes.obj *libmbedcrypto.a:*ccm.obj *libmbedcrypto.a:*ecp.obj) .text*)
        . = ALIGN(4);
        *(EXCLUDE_FILE (*libbee4-internal.a:*.obj *libopenthread-bee4.a:*.obj *libopenthread-ftd.a:*mac_frame.obj *libopenthread-ftd.a:*crypto_platform.obj *libopenthread-ftd.a:*aes_ccm.obj *libopenthread-ftd.a:*aes_ecb.obj *libmbedcrypto.a:*aes.obj *libmbedcrypto.a:*ccm.obj *libmbedcrypto.a:*ecp.obj) .rodata*)
        . = ALIGN(4);
        * (.isr.text)
        * (.app.flash.text)
        * (.app.flash.rodata)
         _etext = .;
         __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        _init_array = .;
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        _einit_array = .;
    } > NS_APP
    __flash_text_load_ad__ = LOADADDR(FLASH_TEXT);

    RAM_VECTOR_TABLE  :
    {
        __ram_vector_exe_ad__ = .;
        KEEP(*(.vectors_table))
        __ram_vector_end_ad__ = .;
    } > RAM_VECTOR_SECTION  AT > NS_APP
    __ram_vector_load_ad__ = LOADADDR(RAM_VECTOR_TABLE);
    __ram_vector_table_length__ = __ram_vector_end_ad__ - __ram_vector_exe_ad__; 

    RAM_VECTOR_TABLE_WRAPPER_NS :
    {
        KEEP(* (.bss.ram_vector_table_wrapper))
    } > RAM_VECTOR_WRAPPER

    ER_IRAM_NS :
    {
        . = ALIGN(0x20);
        __enter_iram_ns_ro_exe_ad__ = .;
        __enter_iram_ns_ro_end_ad__ = .;
        __enter_iram_ns_rw_start__ = .;
        * (.app.ram.text)
        * (.app.ram.data)
        *libbee4-internal.a:*.obj (.text* .rodata*)
        *libopenthread-bee4.a:*.obj (.text* .rodata*)
        *libopenthread-ftd.a:*mac_frame.obj (.text* .rodata*)
        *libopenthread-ftd.a:*crypto_platform.obj (.text* .rodata*)
        *libopenthread-ftd.a:*aes_ccm.obj (.text* .rodata*)
        *libopenthread-ftd.a:*aes_ecb.obj (.text* .rodata*)
        *libmbedcrypto.a:*aes.obj (.text* .rodata*)
        *libmbedcrypto.a:*ccm.obj (.text* .rodata*)
        *libmbedcrypto.a:*ecp.obj (.text* .rodata*)
        *(.data*)
        __enter_iram_ns_rw_end__ = .;
    } > ER_IRAM_NS  AT > NS_APP
    __enter_iram_ns_ro_load_ad__ = LOADADDR(ER_IRAM_NS);
    __enter_iram_ns_ro_length__ = __enter_iram_ns_ro_end_ad__ - __enter_iram_ns_ro_exe_ad__;
    __enter_iram_ns_rw_load_ad__ = LOADADDR(ER_IRAM_NS) + __enter_iram_ns_ro_length__;
    __enter_iram_ns_rw_length__ = __enter_iram_ns_rw_end__ - __enter_iram_ns_rw_start__;

    ER_IRAM_NS_BSS :
    {
        __enter_iram_ns_zi_start__ = .;
        *(.bss*)
        *(COMMON)
        __enter_iram_ns_zi_end__ = .;
    } > ER_IRAM_NS
    __enter_iram_ns_zi_length__ = __enter_iram_ns_zi_end__ - __enter_iram_ns_zi_start__;

    DATA_SRAM_SECTION :
    {
        __data_sram_section_ro_start__ = .;
        __data_sram_section_ro_end__ = .;
        __data_sram_section_rw_start__ = .;
        *(.app.ext.datasram.data)
        __data_sram_section_rw_end__ = .;
        __data_sram_section_zi_start__ = .;
        __data_sram_section_zi_end__ = .;
    } > EXT_DATA_RAM AT > NS_APP
    __data_sram_section_ro_load_ad__ = LOADADDR(DATA_SRAM_SECTION);
    __data_sram_section_ro_length__ = __data_sram_section_ro_end__ - __data_sram_section_ro_start__; 
    __data_sram_section_rw_load_ad__ = LOADADDR(DATA_SRAM_SECTION) + __data_sram_section_ro_length__;
    __data_sram_section_rw_length__ = __data_sram_section_rw_end__ - __data_sram_section_rw_start__; 
    __data_sram_section_rw_exe_ad__ = ADDR(DATA_SRAM_SECTION) + __data_sram_section_ro_length__;
    __data_sram_section_zi_length__ = __data_sram_section_zi_end__ - __data_sram_section_zi_start__; 

    DATA_SRAM_BSS :
    {
        __data_sram_section_zi_start__ = .;
        __data_sram_section_zi_end__ = .;
    } > EXT_DATA_RAM
    __data_sram_section_zi_length__ = __data_sram_section_zi_end__ - __data_sram_section_zi_start__;

    App.trace 0x18100000 :
    {
        *(.TRACE*)
    } > APP_TRACE
}

