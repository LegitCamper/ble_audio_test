use core::cell::RefCell;

use defmt::Debug2Format;
use embassy_nrf::nvmc::Nvmc;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use trouble_audio_example_apps::sink::{BondInformation, BondStore};

const BOND_STORAGE_OFFSET: u32 = 0x0017_C000;
const RECORD_SIZE: usize = 112;

/// A bond store that can discard credentials after a peer forgets this device.
pub trait ClearableBondStore: BondStore {
    /// Erases the persisted bond.
    fn clear(&self);
}

/// Single-peer bond storage backed by the nRF54L15's RRAM.
pub struct RramBondStore<'a, 'd> {
    flash: &'a RefCell<Nvmc<'d>>,
}

impl RramBondStore<'_, '_> {
    fn erase(&self) -> bool {
        let end = BOND_STORAGE_OFFSET + embassy_nrf::nvmc::PAGE_SIZE as u32;
        if self.flash.borrow_mut().erase(BOND_STORAGE_OFFSET, end).is_err() {
            defmt::warn!("failed to erase bond storage");
            false
        } else {
            true
        }
    }
}

impl BondStore for RramBondStore<'_, '_> {
    fn load(&self) -> Option<BondInformation> {
        let mut buffer = [0_u8; RECORD_SIZE];
        if self.flash.borrow_mut().read(BOND_STORAGE_OFFSET, &mut buffer).is_err() {
            defmt::warn!("failed to read bond storage");
            return None;
        }

        let len = usize::from(u16::from_le_bytes([buffer[0], buffer[1]]));
        if len == 0 || len > RECORD_SIZE - 2 {
            return None;
        }
        match postcard::from_bytes(&buffer[2..2 + len]) {
            Ok(bond) => Some(bond),
            Err(error) => {
                defmt::warn!("ignoring unreadable bond: {}", Debug2Format(&error));
                None
            }
        }
    }

    fn save(&self, bond: &BondInformation) {
        let mut encoded = [0_u8; RECORD_SIZE - 2];
        let encoded = match postcard::to_slice(bond, &mut encoded) {
            Ok(encoded) => encoded,
            Err(error) => {
                defmt::warn!("failed to encode bond: {}", Debug2Format(&error));
                return;
            }
        };

        let mut buffer = [0_u8; RECORD_SIZE];
        let Ok(encoded_len) = u16::try_from(encoded.len()) else {
            defmt::warn!("encoded bond is too large: {} bytes", encoded.len());
            return;
        };
        buffer[..2].copy_from_slice(&encoded_len.to_le_bytes());
        buffer[2..2 + encoded.len()].copy_from_slice(encoded);

        if !self.erase() {
            return;
        }
        if self.flash.borrow_mut().write(BOND_STORAGE_OFFSET, &buffer).is_err() {
            defmt::warn!("failed to save bond");
        } else {
            defmt::info!("saved bond");
        }
    }
}

impl ClearableBondStore for RramBondStore<'_, '_> {
    fn clear(&self) {
        if self.erase() {
            defmt::info!("cleared bond storage");
        }
    }
}

/// Creates an RRAM-backed store for the latest bond.
pub fn rram_bond_store<'a, 'd>(flash: &'a RefCell<Nvmc<'d>>) -> RramBondStore<'a, 'd> {
    RramBondStore { flash }
}
