//! Example that transmits serial data using the DMA
//!
//! The first part of the example transmits 10 bytes over serial.
//!
//! The maximum transfer length for DMA1/DMA2 is limited to 65_535 items by
//! hardware. The second part of this example demonstrates splitting a transfer
//! into chunks and using the `next_transfer_with` method to start each part of
//! the transfer.

#![deny(warnings)]
#![no_main]
#![no_std]

use core::sync::atomic::{AtomicBool, Ordering};
use core::{cell::UnsafeCell, mem, mem::MaybeUninit};

use cortex_m_rt::entry;
#[macro_use]
mod utilities;
use stm32h7xx_hal::{pac, prelude::*};

use stm32h7xx_hal::dma::{
    dma::{DmaConfig, StreamsTuple},
    MemoryToPeripheral, Transfer,
};

struct BorrowOnce<T> {
    data: UnsafeCell<T>,
    is_taken: AtomicBool,
}

unsafe impl<T: Sync> Sync for BorrowOnce<T> {}

impl<T> BorrowOnce<T> {
    const fn new(data: T) -> Self {
        Self {
            data: UnsafeCell::new(data),
            is_taken: AtomicBool::new(false),
        }
    }

    /// Calling this more than once WILL lead to a panic
    #[allow(clippy::mut_from_ref)]
    fn get_mut(&self) -> &mut T {
        assert!(
            !self.is_taken.swap(true, Ordering::SeqCst),
            "Already borrowed"
        );

        //NOTE(unsafe) We have just checked that we are the only ones to hold a reference to this
        unsafe { &mut *self.data.get() }
    }
}

use log::info;

// DMA1/DMA2 cannot interact with our stack. Instead, buffers for use with the
// DMA must be placed somewhere that DMA1/DMA2 can access. In this case we use
// AXI SRAM.
//
// The runtime does not initialise these SRAM banks
#[link_section = ".axisram.buffers"]
static SHORT_BUFFER: BorrowOnce<[MaybeUninit<u8>; 10]> =
    BorrowOnce::new([const { MaybeUninit::uninit() }; 10]);

#[link_section = ".axisram.buffers"]
static mut LONG_BUFFER: MaybeUninit<[u32; 0x8000]> = MaybeUninit::uninit();

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(200.MHz())
        .pll1_q_ck(200.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);

    // Acquire the GPIOC peripheral. This also enables the clock for
    // GPIOC in the RCC register.
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    let tx = gpioc.pc10.into_alternate();
    let rx = gpioc.pc11.into_alternate();

    info!("");
    info!("stm32h7xx-hal example - Serial DMA");
    info!("");

    // Configure the serial peripheral.
    let serial = dp
        .USART3
        .serial(
            (tx, rx),
            1_000_000.bps(),
            ccdr.peripheral.USART3,
            &ccdr.clocks,
        )
        .unwrap();

    let (tx, _rx) = serial.split();

    let short_buffer: &mut [MaybeUninit<u8>; 10] = SHORT_BUFFER.get_mut();

    // Initialise the source buffer, without taking any references to
    // uninitialised memory
    let short_buffer: &'static mut [u8; 10] = {
        for (i, value) in short_buffer.iter_mut().enumerate() {
            unsafe {
                value.as_mut_ptr().write(i as u8 + 96); // 0x60, 0x61, 0x62...
            }
        }
        unsafe {
            mem::transmute::<&mut [MaybeUninit<u8>; 10], &mut [u8; 10]>(
                short_buffer,
            )
        }
    };
    // view u32 buffer as u8. Endianess is undefined (little-endian on STM32H7)
    let long_buffer: &'static mut [u8; 0x2_0010] = {
        let buf: &mut [MaybeUninit<u32>; 0x8004] =
            unsafe { &mut *(core::ptr::addr_of_mut!(LONG_BUFFER) as *mut _) };

        for (i, value) in buf.iter_mut().enumerate() {
            unsafe {
                value.as_mut_ptr().write(i as u32);
            }
        }
        unsafe {
            &mut *(core::ptr::addr_of_mut!(LONG_BUFFER) as *mut [u8; 0x2_0010])
        }
    };

    // Setup the DMA transfer on stream 0
    //
    // We need to specify the direction with a type annotation, since DMA
    // transfers both to and from the UART are possible
    let streams = StreamsTuple::new(dp.DMA1, ccdr.peripheral.DMA1);

    let config = DmaConfig::default().memory_increment(true);

    let mut transfer: Transfer<_, _, MemoryToPeripheral, _, _> =
        Transfer::init(streams.0, tx, &mut short_buffer[..], None, config);

    transfer.start(|serial| {
        // This closure runs right after enabling the stream

        // Enable DMA Tx buffer by setting the DMAT bit in the USART_CR3
        // register
        serial.enable_dma_tx();
    });

    // Wait for transfer to complete
    while !transfer.get_transfer_complete_flag() {}

    // Disable DMA Tx once complete (or before)

    info!("Continuing with chunked transfer!");

    // Split the long buffer into chunks: Hardware supports 65_535 max.
    //
    // The last chunk will be length 16, compared to all the others which will
    // be length 32_768.
    for mut chunk in &mut long_buffer.chunks_mut(32_768) {
        // Using `next_transfer_with`
        let _current = transfer
            .next_transfer_with(|mut old, current, remaining| {
                // Check that we really did complete the current transfer
                assert_eq!(remaining, 0);

                mem::swap(&mut old, &mut chunk);

                (old, current)
            })
            .unwrap();

        // Using `next_transfer`: this is equivalent to the above (except the
        // assert) but less flexible
        //transfer.next_transfer(chunk).unwrap();

        // Wait for transfer to complete
        while !transfer.get_transfer_complete_flag() {}
    }

    transfer.pause(|serial| {
        // At this point, the DMA transfer is done, but the data is still in the
        // UART output FIFO. Wait for it to complete
        while !serial.is_txe() {}
    });

    info!("Chunked transfer complete!");

    let (_stream, _serial, _, _) = transfer.free();

    // We could re-use the stream or serial here

    loop {
        cortex_m::asm::nop()
    }
}
