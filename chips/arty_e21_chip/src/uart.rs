// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT

use crate::deferred_call_tasks::DeferredCallTask;
use kernel::deferred_call::DeferredCall;
use kernel::utilities::StaticRef;
use sifive::uart::UartRegisters;

pub const UART0_BASE: StaticRef<UartRegisters> =
    unsafe { StaticRef::new(0x2000_0000 as *const UartRegisters) };

pub static DEFERRED_CALLS: [DeferredCall<DeferredCallTask>; 1] =
    unsafe { [DeferredCall::new(DeferredCallTask::Uart0)] };
