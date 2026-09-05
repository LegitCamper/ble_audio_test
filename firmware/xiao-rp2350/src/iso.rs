//! RP2 EPX isochronous transactions missing from the Embassy host fork.

use core::slice;
use core::sync::atomic::{Ordering, compiler_fence};

use embassy_rp::pac;
use embassy_rp::pac::usb_dpram::vals::EpControlEndpointType;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::host::descriptor::EndpointDescriptor;
use embassy_usb_driver::host::{ChannelError, HostError};
use embassy_usb_driver::{Direction, EndpointType};

const DPRAM_DATA_OFFSET: usize = 0x180;
const DPRAM_DATA_CAPACITY: usize = 1024;
const EPX_CONTROL_OFFSET: usize = 0x100;

/// One direct EPX isochronous endpoint.
#[derive(Clone, Copy)]
pub struct IsoEndpoint {
    device_address: u8,
    endpoint_number: u8,
    direction: Direction,
    max_packet_size: usize,
}

impl IsoEndpoint {
    /// Validates and stores an isochronous endpoint descriptor.
    ///
    /// # Errors
    ///
    /// Returns [`HostError`] for a non-isochronous endpoint or a packet size that cannot fit in
    /// the RP2350 EPX DPRAM buffer.
    pub fn new(device_address: u8, descriptor: EndpointDescriptor) -> Result<Self, HostError> {
        if descriptor.ep_type() != EndpointType::Isochronous {
            return Err(HostError::Other("endpoint is not isochronous"));
        }
        let max_packet_size = usize::from(descriptor.max_packet_size & 0x07ff);
        if max_packet_size == 0 || max_packet_size > DPRAM_DATA_CAPACITY {
            return Err(HostError::InsufficientMemory);
        }
        Ok(Self {
            device_address,
            endpoint_number: descriptor.endpoint_address & 0x0f,
            direction: descriptor.ep_dir(),
            max_packet_size,
        })
    }

    /// Maximum payload accepted by the endpoint in one USB transaction.
    #[must_use]
    pub const fn max_packet_size(&self) -> usize {
        self.max_packet_size
    }

    /// Waits until the host controller advances to a new one-millisecond USB frame.
    pub async fn wait_for_next_sof(&self) {
        let frame = pac::USB.sof_rd().read().count();
        while pac::USB.sof_rd().read().count() == frame {
            Timer::after_micros(20).await;
        }
    }

    /// Fills USB DPRAM in place and sends one DATA0 isochronous OUT transaction.
    ///
    /// # Errors
    ///
    /// Returns [`HostError`] for the wrong endpoint direction, an oversized packet, a bus error,
    /// or a transaction that does not complete within the current USB frame.
    pub fn write_with(&self, packet_len: usize, fill: impl FnOnce(&mut [u8])) -> Result<(), HostError> {
        if self.direction != Direction::Out {
            return Err(HostError::Other("isochronous endpoint is not OUT"));
        }
        if packet_len > self.max_packet_size {
            return Err(ChannelError::BufferOverflow.into());
        }

        self.select();
        compiler_fence(Ordering::SeqCst);
        // SAFETY: the USB peripheral is owned by the core-0 host task. The EPX data region starts
        // at 0x180, and `packet_len` was bounded by the validated endpoint/DPRAM capacity.
        let dpram = unsafe {
            slice::from_raw_parts_mut(pac::USB_DPRAM.as_ptr().cast::<u8>().add(DPRAM_DATA_OFFSET), packet_len)
        };
        fill(dpram);
        compiler_fence(Ordering::SeqCst);

        pac::USB_DPRAM.ep_in_buffer_control(0).write(|control| {
            control.set_available(0, true);
            control.set_pid(0, false);
            control.set_full(0, true);
            control.set_length(0, packet_len as u16);
            control.set_last(0, true);
            control.set_reset(true);
        });
        pac::USB.sie_ctrl().modify(|control| {
            control.set_send_data(true);
            control.set_send_setup(false);
            control.set_receive_data(false);
        });
        self.start_and_wait()
    }

    /// Receives one DATA0 isochronous IN transaction.
    ///
    /// # Errors
    ///
    /// Returns [`HostError`] for the wrong endpoint direction, an undersized destination, a bus
    /// error, or a transaction that does not complete within the current USB frame.
    pub fn read(&self, output: &mut [u8]) -> Result<usize, HostError> {
        if self.direction != Direction::In {
            return Err(HostError::Other("isochronous endpoint is not IN"));
        }
        if output.len() < self.max_packet_size {
            return Err(ChannelError::BufferOverflow.into());
        }

        self.select();
        pac::USB_DPRAM.ep_in_buffer_control(0).write(|control| {
            control.set_available(0, true);
            control.set_pid(0, false);
            control.set_full(0, false);
            control.set_length(0, self.max_packet_size as u16);
            control.set_last(0, true);
            control.set_reset(true);
        });
        pac::USB.sie_ctrl().modify(|control| {
            control.set_send_data(false);
            control.set_send_setup(false);
            control.set_receive_data(true);
        });
        self.start_and_wait()?;

        let received = usize::from(pac::USB_DPRAM.ep_in_buffer_control(0).read().length(0));
        if received > output.len() {
            return Err(ChannelError::BufferOverflow.into());
        }
        compiler_fence(Ordering::SeqCst);
        // SAFETY: the USB peripheral is exclusively driven by core 0, and hardware-reported
        // `received` was checked against the caller buffer and the configured endpoint capacity.
        let dpram =
            unsafe { slice::from_raw_parts(pac::USB_DPRAM.as_ptr().cast::<u8>().add(DPRAM_DATA_OFFSET), received) };
        output[..received].copy_from_slice(dpram);
        compiler_fence(Ordering::SeqCst);
        Ok(received)
    }

    fn select(&self) {
        pac::USB.inte().modify(|interrupts| {
            interrupts.set_trans_complete(false);
            interrupts.set_stall(false);
            interrupts.set_error_rx_timeout(false);
            interrupts.set_error_rx_overflow(false);
        });
        clear_transaction_status();
        pac::USB.addr_endp().write(|address| {
            address.set_address(self.device_address);
            address.set_endpoint(self.endpoint_number);
        });

        // SAFETY: 0x100 is the RP2350 USB DPRAM EPX control register omitted by the SVD. This is
        // the same address and register type used by the pinned Embassy fork's private helper.
        let endpoint_control: pac::common::Reg<pac::usb_dpram::regs::EpControl, pac::common::RW> =
            unsafe { pac::common::Reg::from_ptr(pac::USB_DPRAM.as_ptr().cast::<u8>().add(EPX_CONTROL_OFFSET).cast()) };
        endpoint_control.write(|control| {
            control.set_enable(true);
            control.set_double_buffered(false);
            control.set_interrupt_per_buff(false);
            control.set_endpoint_type(EpControlEndpointType::ISOCHRONOUS);
            control.set_buffer_address(DPRAM_DATA_OFFSET as u16);
        });
        pac::USB.sie_ctrl().modify(|control| control.set_preamble_en(false));
    }

    fn start_and_wait(&self) -> Result<(), HostError> {
        cortex_m::asm::delay(12);
        pac::USB.sie_ctrl().modify(|control| control.set_start_trans(true));
        let deadline = Instant::now() + Duration::from_micros(900);

        loop {
            let status = pac::USB.sie_status().read();
            if status.trans_complete() {
                clear_transaction_status();
                return Ok(());
            }
            if status.stall_rec() {
                clear_transaction_status();
                return Err(ChannelError::Stall.into());
            }
            if status.rx_overflow() {
                clear_transaction_status();
                return Err(ChannelError::BufferOverflow.into());
            }
            if status.rx_timeout() || Instant::now() >= deadline {
                clear_transaction_status();
                return Err(ChannelError::Timeout.into());
            }
            cortex_m::asm::nop();
        }
    }
}

fn clear_transaction_status() {
    pac::USB.sie_status().write(|status| {
        status.set_trans_complete(true);
        status.set_stall_rec(true);
        status.set_rx_overflow(true);
        status.set_rx_timeout(true);
    });
}
