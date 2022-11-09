// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT

use kernel::utilities::StaticRef;
pub use lowrisc::usbdev::Usb;
use lowrisc::usbdev::UsbRegisters;

pub const USB0_BASE: StaticRef<UsbRegisters> =
    unsafe { StaticRef::new(0x4032_0000 as *const UsbRegisters) };
