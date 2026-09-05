use alloc::vec::Vec;
use core::cell::RefCell;

use embassy_nrf::nvmc::Nvmc;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use trouble_audio_example_apps::bond_store::EncodedBondStore;

const BOND_STORAGE_OFFSET: u32 = 0x0017_C000;
const RECORD_SIZE: usize = 112;

pub fn rram_bond_store<'a>(
    flash: &'a RefCell<Nvmc<'_>>,
) -> EncodedBondStore<impl Fn() -> Option<Vec<u8>> + 'a, impl Fn(&[u8]) + 'a> {
    EncodedBondStore::new(
        move || {
            let mut buffer = [0_u8; RECORD_SIZE];
            if flash.borrow_mut().read(BOND_STORAGE_OFFSET, &mut buffer).is_err() {
                defmt::warn!("failed to read bond storage");
                return None;
            }

            let len = usize::from(u16::from_le_bytes([buffer[0], buffer[1]]));
            if len == 0 || len > RECORD_SIZE - 2 {
                return None;
            }
            Some(buffer[2..2 + len].to_vec())
        },
        move |encoded: &[u8]| {
            if encoded.len() > RECORD_SIZE - 2 {
                defmt::warn!("encoded bond is too large: {} bytes", encoded.len());
                return;
            }

            let mut buffer = [0_u8; RECORD_SIZE];
            let Ok(encoded_len) = u16::try_from(encoded.len()) else {
                return;
            };
            buffer[..2].copy_from_slice(&encoded_len.to_le_bytes());
            buffer[2..2 + encoded.len()].copy_from_slice(encoded);

            let mut storage = flash.borrow_mut();
            let end = BOND_STORAGE_OFFSET + embassy_nrf::nvmc::PAGE_SIZE as u32;
            if storage.erase(BOND_STORAGE_OFFSET, end).is_err() {
                defmt::warn!("failed to erase bond storage");
                return;
            }
            if storage.write(BOND_STORAGE_OFFSET, &buffer).is_err() {
                defmt::warn!("failed to save bond");
            } else {
                defmt::info!("saved bond");
            }
        },
    )
}
