// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: MIT OR Apache-2.0

use kernel::utilities::registers::register_bitfields;

register_bitfields![usize,
    pub mscratch [
        scratch OFFSET(0) NUMBITS(crate::XLEN) []
    ]
];
