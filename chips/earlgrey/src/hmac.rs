// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT

use kernel::utilities::StaticRef;
use lowrisc::hmac::HmacRegisters;

pub const HMAC0_BASE: StaticRef<HmacRegisters> =
    unsafe { StaticRef::new(0x4111_0000 as *const HmacRegisters) };
