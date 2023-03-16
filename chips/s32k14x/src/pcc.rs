use kernel::platform::chip::ClockInterface;
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, register_structs, ReadOnly, ReadWrite};
use kernel::utilities::StaticRef;

pub const PCC_PCCn_COUNT: usize = 116;

#[repr(C)]
struct PccRegisters {
    /// Channel configuration registers, one per channel.
    PCCn: [ReadWrite<u32, ChannelConfiguration::Register>; PCC_PCCn_COUNT],
}

const PCC_BASE: StaticRef<PccRegisters> =
    unsafe { StaticRef::new(0x4006_5000 as *const PccRegisters) };

registers::register_bitfields![u32,
    ChannelConfiguration [
        PR OFFSET(31) NUMBITS(1) [],
        CGC OFFSET(30) NUMBITS(1) [],
    ]
];
