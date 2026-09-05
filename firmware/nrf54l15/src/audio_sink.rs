//! A 48 kHz LE Audio unicast sink peripheral.
//!
//! Both Sink ASEs are forwarded as a timestamp-paired left/right stream for decoding on RP2350.

use alloc::vec;

use bt_hci::cmd::le::{LeReadLocalSupportedFeatures, LeSetHostFeature};
use bt_hci::controller::{ControllerCmdAsync, ControllerCmdSync};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use heapless::Vec as HVec;
use trouble_audio::cis::CisManager;
use trouble_audio::prelude::*;
use trouble_audio_example_apps::basic_audio_sink::{ADDRESS, MAX_ASES};
use trouble_audio_example_apps::sink::{BondStore, PeripheralConfig, run_peripheral};
use trouble_host::prelude::*;

/// Max number of simultaneous BLE connections; the ASCS server tracks one at a time.
const CONNECTIONS_MAX: usize = 1;
/// Signalling + ATT + one connection-oriented channel.
const L2CAP_CHANNELS_MAX: usize = 3;

/// Runs the 48 kHz stereo audio sink peripheral forever on the given controller.
///
/// `cis_manager` is caller-owned so the caller can concurrently drain
/// [`CisManager::receive_lc3`]; run this alongside whatever does that.
pub async fn run<C>(controller: C, cis_manager: &CisManager<NoopRawMutex, MAX_ASES>, bond_store: &dyn BondStore) -> !
where
    C: Controller
        + ControllerCmdAsync<LeAcceptCisRequest>
        + ControllerCmdSync<LeRejectCisRequest>
        + for<'a> ControllerCmdSync<LeSetupIsoDataPath<'a>>
        + ControllerCmdSync<LeRemoveIsoDataPath>
        + ControllerCmdSync<LeSetHostFeature>
        + ControllerCmdSync<LeReadLocalSupportedFeatures>,
{
    let config = PeripheralConfig {
        device_name: b"Ble Audio Sink",
        appearance: appearance::audio_sink::GENERIC_AUDIO_SINK,
        sink_pac: Some(PAC::new(&[PACRecord {
            codec_id: CodecId::default(), // LC3
            codec_specific_capabilities: vec![
                CodecSpecificCapabilities::SupportedSamplingFrequencies(SupportedSamplingFrequencies::new(&[
                    SamplingFrequency::Hz48000,
                ])),
                // Both of the following are mandatory per PACS: without them a central cannot
                // derive any valid stream configuration from this record at all.
                CodecSpecificCapabilities::SupportedFrameDurations(SupportedFrameDurations::default()), // 10 ms only
                // Covers the 48 kHz/10 ms operating points (48_1 is 75 octets, 48_2 is 100).
                CodecSpecificCapabilities::SupportedOctetsPerCodecFrame(OctetsPerCodecFrame::new(26, 155)),
            ],
            metadata: vec![],
        }])),
        // One location per Sink ASE. The declared locations and the ASE count must match, or a
        // central that picks a two-CIS stereo group never sees "all ASEs configured".
        sink_audio_locations: Some(AudioLocation::FrontLeft | AudioLocation::FrontRight),
        source_pac: None,
        source_audio_locations: None,
        supported_audio_contexts: AudioContexts {
            sink_contexts: ContextType::Media | ContextType::Conversational,
            source_contexts: ContextType::empty(),
        },
        available_audio_contexts: AudioContexts {
            sink_contexts: ContextType::Media | ContextType::Conversational,
            source_contexts: ContextType::empty(),
        },
    };

    let mut ases = HVec::new();
    // ASCS reserves ASE_ID 0x00 for error responses; server-assigned IDs must be nonzero.
    let _ = ases.push(AseType::Sink(Ase::new(1)));
    let _ = ases.push(AseType::Sink(Ase::new(2)));

    run_peripheral::<C, NoopRawMutex, MAX_ASES, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX>(
        controller,
        Address::random(ADDRESS),
        // No display or keyboard on an audio sink: JustWorks pairing.
        IoCapabilities::NoInputNoOutput,
        config,
        ases,
        cis_manager,
        Some(bond_store),
    )
    .await
}
