// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Component for Digital to Analog Converters (DAC).
//!
//! Usage
//! -----
//! ```rust
//! let dac = components::dac::DacComponent::new(&peripherals.dac)
//!      .finalize(components::dac_component_static!());
//! ```

use capsules::dac::Dac;
use core::mem::MaybeUninit;
use kernel::component::Component;
use kernel::hil;

#[macro_export]
macro_rules! dac_component_static {
    () => {{
        kernel::static_buf!(capsules::dac::Dac<'static>)
    };};
}

pub struct DacComponent {
    dac: &'static dyn hil::dac::DacChannel,
}

impl DacComponent {
    pub fn new(dac: &'static dyn hil::dac::DacChannel) -> Self {
        Self { dac }
    }
}

impl Component for DacComponent {
    type StaticInput = &'static mut MaybeUninit<Dac<'static>>;
    type Output = &'static Dac<'static>;

    fn finalize(self, s: Self::StaticInput) -> Self::Output {
        s.write(Dac::new(self.dac))
    }
}
