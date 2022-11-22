// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Definition of Deferred Call tasks.
//!
//! Deferred calls allow peripheral drivers to register pseudo interrupts.
//! These are the definitions of which deferred calls this chip needs.

use core::convert::Into;
use core::convert::TryFrom;

/// A type of task to defer a call for
#[derive(Copy, Clone)]
pub enum Task {
    Flashcalw = 0,
    CRCCU = 1,
    Usart0 = 2,
    Usart1 = 3,
    Usart2 = 4,
    Usart3 = 5,
}

impl TryFrom<usize> for Task {
    type Error = ();

    fn try_from(value: usize) -> Result<Task, ()> {
        match value {
            0 => Ok(Task::Flashcalw),
            1 => Ok(Task::CRCCU),
            2 => Ok(Task::Usart0),
            3 => Ok(Task::Usart1),
            4 => Ok(Task::Usart2),
            5 => Ok(Task::Usart3),
            _ => Err(()),
        }
    }
}

impl Into<usize> for Task {
    fn into(self) -> usize {
        self as usize
    }
}
