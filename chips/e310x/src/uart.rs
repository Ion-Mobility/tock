// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! UART instantiation.

use crate::deferred_call_tasks::DeferredCallTask;
use kernel::deferred_call::DeferredCall;
use kernel::utilities::StaticRef;
use sifive::uart::UartRegisters;

pub const UART0_BASE: StaticRef<UartRegisters> =
    unsafe { StaticRef::new(0x1001_3000 as *const UartRegisters) };

pub const UART1_BASE: StaticRef<UartRegisters> =
    unsafe { StaticRef::new(0x1002_3000 as *const UartRegisters) };

pub static DEFERRED_CALLS: [DeferredCall<DeferredCallTask>; 2] = unsafe {
    [
        DeferredCall::new(DeferredCallTask::Uart0),
        DeferredCall::new(DeferredCallTask::Uart1),
    ]
};
