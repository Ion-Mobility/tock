// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: MIT OR Apache-2.0

use crate::tests::run_kernel_op;
use crate::PERIPHERALS;
use core::cell::Cell;
#[allow(unused_imports)]
use kernel::hil::spi::{ClockPhase, ClockPolarity};
use kernel::hil::spi::{SpiMaster, SpiMasterClient};
use kernel::static_init;
use kernel::utilities::cells::TakeCell;
use kernel::{debug, ErrorCode};

struct SpiHostCallback {
    transfer_done: Cell<bool>,
    tx_len: Cell<usize>,
    tx_data: TakeCell<'static, [u8]>,
    rx_data: TakeCell<'static, [u8]>,
}

impl<'a> SpiHostCallback {
    fn new(tx_data: &'static mut [u8], rx_data: &'static mut [u8]) -> Self {
        SpiHostCallback {
            transfer_done: Cell::new(false),
            tx_len: Cell::new(0),
            tx_data: TakeCell::new(tx_data),
            rx_data: TakeCell::new(rx_data),
        }
    }

    fn reset(&self) {
        self.transfer_done.set(false);
        self.tx_len.set(0);
    }
}

impl<'a> SpiMasterClient for SpiHostCallback {
    fn read_write_done(
        &self,
        tx_data: &'static mut [u8],
        rx_done: Option<&'static mut [u8]>,
        tx_len: usize,
        rc: Result<(), ErrorCode>,
    ) {
        // Transfer Complete
        assert_eq!(rc, Ok(()));
        assert_eq!(tx_len, self.tx_len.get());

        // Capture Buffers
        match rx_done {
            Some(rx_buf) => {
                self.rx_data.replace(rx_buf);
            }
            None => {
                panic!("RX Buffer Lost");
            }
        }

        self.tx_data.replace(tx_data);

        if self.tx_len.get() == tx_len {
            self.transfer_done.set(true);
        }
    }
}

unsafe fn static_init_test_cb() -> &'static SpiHostCallback {
    let rx_data = static_init!([u8; 32], [0; 32]);

    let tx_data = static_init!(
        [u8; 32],
        [
            0xdc, 0x55, 0x51, 0x5e, 0x30, 0xac, 0x50, 0xc7, 0x65, 0xbd, 0xe, 0x2, 0x82, 0xf7, 0x8b,
            0xe1, 0xef, 0xd1, 0xb, 0xdc, 0xa8, 0xba, 0xe1, 0xfa, 0x11, 0x3f, 0xf6, 0xeb, 0xaf,
            0x58, 0x57, 0x40,
        ]
    );

    static_init!(SpiHostCallback, SpiHostCallback::new(tx_data, rx_data))
}

unsafe fn static_init_test_partial_cb() -> &'static SpiHostCallback {
    let rx_data = static_init!([u8; 513], [0; 513]);
    // Buffer Size to exceed TXFIFO (Force partial transfers)
    let tx_data = static_init!(
        [u8; 513],
        [
            0xdc, 0x55, 0x51, 0x5e, 0x30, 0xac, 0x50, 0xc7, 0x65, 0xbd, 0xe, 0x2, 0x82, 0xf7, 0x8b,
            0xe1, 0xef, 0xd1, 0xb, 0xdc, 0xa8, 0xba, 0xe1, 0xfa, 0x11, 0x3f, 0xf6, 0xeb, 0xaf,
            0x58, 0x57, 0x40, 0xdc, 0x55, 0x51, 0x5e, 0x30, 0xac, 0x50, 0xc7, 0x65, 0xbd, 0xe, 0x2,
            0x82, 0xf7, 0x8b, 0xe1, 0xef, 0xd1, 0xb, 0xdc, 0xa8, 0xba, 0xe1, 0xfa, 0x11, 0x3f,
            0xf6, 0xeb, 0xaf, 0x58, 0x57, 0x40, 0xdc, 0x55, 0x51, 0x5e, 0x30, 0xac, 0x50, 0xc7,
            0x65, 0xbd, 0xe, 0x2, 0x82, 0xf7, 0x8b, 0xe1, 0xef, 0xd1, 0xb, 0xdc, 0xa8, 0xba, 0xe1,
            0xfa, 0x11, 0x3f, 0xf6, 0xeb, 0xaf, 0x58, 0x57, 0x40, 0xdc, 0x55, 0x51, 0x5e, 0x30,
            0xac, 0x50, 0xc7, 0x65, 0xbd, 0xe, 0x2, 0x82, 0xf7, 0x8b, 0xe1, 0xef, 0xd1, 0xb, 0xdc,
            0xa8, 0xba, 0xe1, 0xfa, 0x11, 0x3f, 0xf6, 0xeb, 0xaf, 0x58, 0x57, 0x40, 0xdc, 0x55,
            0x51, 0x5e, 0x30, 0xac, 0x50, 0xc7, 0x65, 0xbd, 0xe, 0x2, 0x82, 0xf7, 0x8b, 0xe1, 0xef,
            0xd1, 0xb, 0xdc, 0xa8, 0xba, 0xe1, 0xfa, 0x11, 0x3f, 0xf6, 0xeb, 0xaf, 0x58, 0x57,
            0x40, 0xdc, 0x55, 0x51, 0x5e, 0x30, 0xac, 0x50, 0xc7, 0x65, 0xbd, 0xe, 0x2, 0x82, 0xf7,
            0x8b, 0xe1, 0xef, 0xd1, 0xb, 0xdc, 0xa8, 0xba, 0xe1, 0xfa, 0x11, 0x3f, 0xf6, 0xeb,
            0xaf, 0x58, 0x57, 0x40, 0xdc, 0x55, 0x51, 0x5e, 0x30, 0xac, 0x50, 0xc7, 0x65, 0xbd,
            0xe, 0x2, 0x82, 0xf7, 0x8b, 0xe1, 0xef, 0xd1, 0xb, 0xdc, 0xa8, 0xba, 0xe1, 0xfa, 0x11,
            0x3f, 0xf6, 0xeb, 0xaf, 0x58, 0x57, 0x40, 0xdc, 0x55, 0x51, 0x5e, 0x30, 0xac, 0x50,
            0xc7, 0x65, 0xbd, 0xe, 0x2, 0x82, 0xf7, 0x8b, 0xe1, 0xef, 0xd1, 0xb, 0xdc, 0xa8, 0xba,
            0xe1, 0xfa, 0x11, 0x3f, 0xf6, 0xeb, 0xaf, 0x58, 0x57, 0x40, 0xdc, 0x55, 0x51, 0x5e,
            0x30, 0xac, 0x50, 0xc7, 0x65, 0xbd, 0xe, 0x2, 0x82, 0xf7, 0x8b, 0xe1, 0xef, 0xd1, 0xb,
            0xdc, 0xa8, 0xba, 0xe1, 0xfa, 0x11, 0x3f, 0xf6, 0xeb, 0xaf, 0x58, 0x57, 0x40, 0xdc,
            0x55, 0x51, 0x5e, 0x30, 0xac, 0x50, 0xc7, 0x65, 0xbd, 0xe, 0x2, 0x82, 0xf7, 0x8b, 0xe1,
            0xef, 0xd1, 0xb, 0xdc, 0xa8, 0xba, 0xe1, 0xfa, 0x11, 0x3f, 0xf6, 0xeb, 0xaf, 0x58,
            0x57, 0x40, 0xdc, 0x55, 0x51, 0x5e, 0x30, 0xac, 0x50, 0xc7, 0x65, 0xbd, 0xe, 0x2, 0x82,
            0xf7, 0x8b, 0xe1, 0xef, 0xd1, 0xb, 0xdc, 0xa8, 0xba, 0xe1, 0xfa, 0x11, 0x3f, 0xf6,
            0xeb, 0xaf, 0x58, 0x57, 0x40, 0xdc, 0x55, 0x51, 0x5e, 0x30, 0xac, 0x50, 0xc7, 0x65,
            0xbd, 0xe, 0x2, 0x82, 0xf7, 0x8b, 0xe1, 0xef, 0xd1, 0xb, 0xdc, 0xa8, 0xba, 0xe1, 0xfa,
            0x11, 0x3f, 0xf6, 0xeb, 0xaf, 0x58, 0x57, 0x40, 0xdc, 0x55, 0x51, 0x5e, 0x30, 0xac,
            0x50, 0xc7, 0x65, 0xbd, 0xe, 0x2, 0x82, 0xf7, 0x8b, 0xe1, 0xef, 0xd1, 0xb, 0xdc, 0xa8,
            0xba, 0xe1, 0xfa, 0x11, 0x3f, 0xf6, 0xeb, 0xaf, 0x58, 0x57, 0x40, 0xdc, 0x55, 0x51,
            0x5e, 0x30, 0xac, 0x50, 0xc7, 0x65, 0xbd, 0xe, 0x2, 0x82, 0xf7, 0x8b, 0xe1, 0xef, 0xd1,
            0xb, 0xdc, 0xa8, 0xba, 0xe1, 0xfa, 0x11, 0x3f, 0xf6, 0xeb, 0xaf, 0x58, 0x57, 0x40,
            0xdc, 0x55, 0x51, 0x5e, 0x30, 0xac, 0x50, 0xc7, 0x65, 0xbd, 0xe, 0x2, 0x82, 0xf7, 0x8b,
            0xe1, 0xef, 0xd1, 0xb, 0xdc, 0xa8, 0xba, 0xe1, 0xfa, 0x11, 0x3f, 0xf6, 0xeb, 0xaf,
            0x58, 0x57, 0x40, 0xdc, 0x55, 0x51, 0x5e, 0x30, 0xac, 0x50, 0xc7, 0x65, 0xbd, 0xe, 0x2,
            0x82, 0xf7, 0x8b, 0xe1, 0xef, 0xd1, 0xb, 0xdc, 0xa8, 0xba, 0xe1, 0xfa, 0x11, 0x3f,
            0xf6, 0xeb, 0xaf, 0x58, 0x57, 0x40, 0x40,
        ]
    );

    static_init!(SpiHostCallback, SpiHostCallback::new(tx_data, rx_data))
}

/// Tests transferring a data set that exceeds the TXFIFO (256)
/// The driver must do 3 transfers (256, 256, 1) to transfer the full 513 byte
/// dataset. This tests partial transfers and continued offset write outs.
#[test_case]
fn spi_host_transfer_partial() {
    let perf = unsafe { PERIPHERALS.unwrap() };
    let spi_host = &perf.iom2;

    let cb = unsafe { static_init_test_partial_cb() };

    debug!("[SPI] Setup iom2 partial_transfer... ");
    run_kernel_op(100);

    spi_host.set_client(cb);
    cb.reset();

    let tx = cb.tx_data.take().unwrap();
    let rx = cb.rx_data.take().unwrap();
    cb.tx_len.set(tx.len());

    // Set iom2 Configs
    spi_host
        .specify_chip_select(
            &perf.gpio_port[11], // A5
        )
        .ok();
    spi_host.set_rate(100000).ok();
    spi_host.set_polarity(ClockPolarity::IdleLow).ok();
    spi_host.set_phase(ClockPhase::SampleLeading).ok();

    assert_eq!(
        spi_host.read_write_bytes(tx, Some(rx), cb.tx_len.get()),
        Ok(())
    );
    run_kernel_op(5000);

    assert_eq!(cb.transfer_done.get(), true);

    run_kernel_op(100);
    debug!("    [ok]");
    run_kernel_op(100);
}

/// Tests two single transfers that do not exceed the TXFIFO
/// The second test, is to ensure that the driver is left in a clean state
/// after a transfer (reset internal offsets and counts etc...)
#[test_case]
fn spi_host_transfer_single() {
    let perf = unsafe { PERIPHERALS.unwrap() };
    let spi_host = &perf.iom2;

    let cb = unsafe { static_init_test_cb() };

    debug!("[SPI] Setup iom2 transfers... ");
    run_kernel_op(100);
    spi_host.set_client(cb);
    cb.reset();

    let tx = cb.tx_data.take().unwrap();
    let rx = cb.rx_data.take().unwrap();
    cb.tx_len.set(tx.len());

    // Set iom2 Configs
    // spi_host.specify_chip_select(0).ok();
    spi_host.set_rate(100000).ok();
    spi_host.set_polarity(ClockPolarity::IdleLow).ok();
    spi_host.set_phase(ClockPhase::SampleLeading).ok();

    assert_eq!(
        spi_host.read_write_bytes(tx, Some(rx), cb.tx_len.get()),
        Ok(())
    );
    run_kernel_op(5000);

    assert_eq!(cb.transfer_done.get(), true);

    run_kernel_op(100);
    debug!("    [ok]");
    run_kernel_op(100);

    debug!("[SPI] Setup iom2 transfer...x2 ");
    run_kernel_op(100);

    cb.reset();

    let tx2 = cb.tx_data.take().unwrap();
    let rx2 = cb.rx_data.take().unwrap();
    cb.tx_len.set(tx2.len());

    assert_eq!(
        spi_host.read_write_bytes(tx2, Some(rx2), cb.tx_len.get()),
        Ok(())
    );
    run_kernel_op(5000);

    assert_eq!(cb.transfer_done.get(), true);

    run_kernel_op(100);
    debug!("    [ok]");
    run_kernel_op(100);
}
