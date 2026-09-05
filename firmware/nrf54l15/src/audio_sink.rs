//! A 48 kHz LE Audio unicast sink peripheral.
//!
//! Both Sink ASEs are forwarded as a timestamp-paired left/right stream for decoding on RP2350.
//!
//! The GATT table and connection loop are built here rather than through
//! `trouble_audio_example_apps::sink::run_peripheral`, because that helper builds its server from
//! a `PeripheralConfig` that has no Volume Control slot. This sink has to expose VCS so the phone
//! owns the volume slider, and has to observe each Volume Control Point write so the setting can
//! be forwarded to the USB bridge.

use alloc::vec;

use ble_audio_link::VolumeCommand;
use bt_hci::cmd::le::{LeReadLocalSupportedFeatures, LeSetHostFeature};
use bt_hci::controller::{ControllerCmdAsync, ControllerCmdSync};
use defmt::Debug2Format;
use embassy_futures::select::{Either4, select4};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Duration;
use heapless::Vec as HVec;
use trouble_audio::cis::{self, CisManager};
use trouble_audio::prelude::*;
use trouble_audio::vcs::{Mute, VcsStorage, VolumeFlags, VolumeState};
use trouble_audio_example_apps::basic_audio_sink::{ADDRESS, MAX_ASES};
use trouble_audio_example_apps::sink::BondStore;
use trouble_host::prelude::*;

/// Max number of simultaneous BLE connections; the ASCS server tracks one at a time.
const CONNECTIONS_MAX: usize = 1;
/// Signalling + ATT + one connection-oriented channel.
const L2CAP_CHANNELS_MAX: usize = 3;
/// Name advertised and exposed through the GAP Device Name characteristic.
const DEVICE_NAME: &[u8] = b"Ble Audio Sink";
/// Volume this sink reports before a client sets one.
///
/// Roughly half scale, so a freshly paired phone that renders before touching its slider is
/// audible without being startling.
const INITIAL_VOLUME: u8 = 128;
/// Amount one relative Volume Control Point step moves the setting, giving 16 steps across the
/// scale, which is close to what phone volume rockers expect.
const VOLUME_STEP: u8 = 16;
/// Time spent targeting a saved peer before returning to discoverable advertising.
const DIRECTED_RECONNECT_TIMEOUT: Duration = Duration::from_secs(5);

/// Attribute backing store for every service this sink exposes.
///
/// Held in one struct so the individual services can borrow disjoint fields for the lifetime of
/// the server.
#[derive(Default)]
struct SinkStorage {
    pacs: PacsStorage,
    ascs: AscsStorage<MAX_ASES>,
    vcs: VcsStorage,
}

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
    let appearance = appearance::audio_sink::GENERIC_AUDIO_SINK;
    let sink_pac = PAC::new(&[PACRecord {
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
    }]);
    // One location per Sink ASE. The declared locations and the ASE count must match, or a
    // central that picks a two-CIS stereo group never sees "all ASEs configured".
    let sink_audio_locations = AudioLocation::FrontLeft | AudioLocation::FrontRight;
    let audio_contexts = AudioContexts {
        sink_contexts: ContextType::Media | ContextType::Conversational,
        source_contexts: ContextType::empty(),
    };

    let mut ases = HVec::new();
    // ASCS reserves ASE_ID 0x00 for error responses; server-assigned IDs must be nonzero.
    let _ = ases.push(AseType::Sink(Ase::new(1)));
    let _ = ases.push(AseType::Sink(Ase::new(2)));

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(Address::random(ADDRESS))
        // No display or keyboard on an audio sink: JustWorks pairing.
        .set_io_capabilities(IoCapabilities::NoInputNoOutput)
        .build();
    let mut bonded_peer = bond_store.load().map(|bond| {
        let peer = bond.identity.addr;
        let _ = stack.add_bond_information(bond);
        defmt::info!("loaded saved peer for directed reconnect: {}", peer);
        peer
    });
    let mut runner = stack.runner();
    let mut peripheral = stack.peripheral();

    // Encoded once and reused across reconnects: the payload never changes for the life of this
    // call. VCS is deliberately left out of the advertised UUIDs, which stays byte-identical to
    // the advertisement this sink was brought up with; the list is already marked incomplete and
    // a client discovers Volume Control after connecting.
    let mut advertiser_data = [0; 31];
    let adv_data_len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::IncompleteServiceUuids16(&[
                service::PUBLISHED_AUDIO_CAPABILITIES.to_le_bytes(),
                service::AUDIO_STREAM_CONTROL.to_le_bytes(),
                service::COMMON_AUDIO.to_le_bytes(),
            ]),
            AdStructure::CompleteLocalName(DEVICE_NAME),
        ],
        &mut advertiser_data[..],
    )
    .expect("static advertising data always fits in 31 bytes");
    let advertiser_data = &advertiser_data[..adv_data_len];

    let mut storage = SinkStorage::default();
    let server = ServerBuilder::<MAX_ASES, CONNECTIONS_MAX, NoopRawMutex>::new(DEVICE_NAME, &appearance)
        .add_pacs(
            Some(&sink_pac),
            Some(&sink_audio_locations),
            None,
            None,
            &audio_contexts,
            &audio_contexts,
            &mut storage.pacs,
        )
        .add_ascs(ases, &mut storage.ascs)
        // The flags stay empty because this sink does not persist volume across power cycles, so
        // it must keep reporting its factory default.
        .add_vcs(
            VolumeState {
                volume_setting: INITIAL_VOLUME,
                mute: Mute::NotMuted,
                change_counter: 0,
            },
            VolumeFlags::empty(),
            VOLUME_STEP,
            &mut storage.vcs,
        )
        .add_cis_manager(cis_manager)
        .build();

    select4(
        async {
            loop {
                if let Err(error) = runner.run_with_handler(cis_manager).await {
                    defmt::warn!("host runner error: {}", Debug2Format(&error));
                }
            }
        },
        cis::drive_cis(&stack, cis_manager),
        async {
            cis::enable_cis_host_support(&stack).await;
            core::future::pending::<()>().await
        },
        async {
            loop {
                let conn = match advertise(advertiser_data, bonded_peer, &mut peripheral, &server).await {
                    Ok(conn) => conn,
                    Err(error) => {
                        defmt::warn!("advertise error: {}", Debug2Format(&error));
                        continue;
                    }
                };
                defmt::info!("connected");
                // A connection is not bondable by default, and must be marked as such before
                // pairing starts - otherwise `PairingComplete` always reports `bond: None` (a
                // temporary key only), even if the peer requests bonding.
                let _ = conn.raw().set_bondable(true);

                // Publish the current setting up front so the bridge starts from the same volume
                // this server reports, rather than from whatever the DAC powered up at.
                let mut last_volume = None;
                publish_volume(&server, &mut last_volume);

                loop {
                    let event = match select4(
                        conn.next(),
                        cis_manager.next_streaming_ase(),
                        cis_manager.next_released_ase(),
                        cis_manager.next_qos_configured_ase(),
                    )
                    .await
                    {
                        Either4::First(event) => event,
                        Either4::Second(ase_id) => {
                            server.notify_ase_streaming(&conn, ase_id).await;
                            continue;
                        }
                        Either4::Third(ase_id) => {
                            server.notify_ase_released(&conn, ase_id).await;
                            continue;
                        }
                        Either4::Fourth(ase_id) => {
                            server.notify_ase_qos_configured(&conn, ase_id).await;
                            continue;
                        }
                    };
                    match event {
                        GattConnectionEvent::Disconnected { reason } => {
                            defmt::info!("disconnected: {}", Debug2Format(&reason));
                            server.reset_connection();
                            break;
                        }
                        GattConnectionEvent::Gatt { event } => {
                            server.handle(&conn, event).await;
                            // `handle` applies a Volume Control Point write to the Volume State
                            // characteristic itself, so the new setting is readable right after
                            // it returns.
                            publish_volume(&server, &mut last_volume);
                        }
                        GattConnectionEvent::PairingComplete {
                            security_level,
                            bond: Some(bond),
                        } => {
                            defmt::info!("pairing complete, security_level={}", Debug2Format(&security_level));
                            // The pairing flow already stores `bond` for LTK lookup, but only
                            // `add_bond_information` also queues the controller's resolving list
                            // to be updated with the peer's IRK - without it, a peer using a
                            // rotating private address can pair successfully and then never be
                            // recognized on its next reconnect.
                            let _ = stack.add_bond_information(bond.clone());
                            bond_store.save(&bond);
                            bonded_peer = Some(bond.identity.addr);
                        }
                        GattConnectionEvent::PairingComplete {
                            security_level,
                            bond: None,
                        } => {
                            defmt::warn!(
                                "pairing complete but NOT bonded (security_level={}); reconnects will re-pair",
                                Debug2Format(&security_level)
                            );
                        }
                        GattConnectionEvent::PairingFailed(error) => {
                            defmt::warn!("pairing failed: {}", Debug2Format(&error));
                        }
                        _ => {}
                    }
                }
            }
        },
    )
    .await;

    unreachable!("every branch above loops forever")
}

/// Forwards the server's volume to the USB bridge when it has changed.
///
/// Reading it back from the characteristic rather than tracking the control point operations
/// separately means the value sent is always the one the spec state machine actually settled on,
/// including its clamping of relative steps.
fn publish_volume(server: &Server<'_, MAX_ASES, CONNECTIONS_MAX, NoopRawMutex>, last: &mut Option<VolumeState>) {
    let Some(vcs) = server.vcs() else {
        return;
    };
    let Ok(state) = vcs.volume_state().get(&server.server) else {
        defmt::warn!("could not read the volume state characteristic");
        return;
    };
    // The change counter moves on every write, so compare only the fields that affect rendering.
    let changed = last.is_none_or(|last| last.volume_setting != state.volume_setting || last.mute != state.mute);
    if !changed {
        return;
    }
    *last = Some(state);
    crate::send_volume(VolumeCommand {
        level: state.volume_setting,
        muted: state.mute == Mute::Muted,
    });
}

async fn advertise<'values, 'server, C: Controller>(
    adv_data: &[u8],
    bonded_peer: Option<Address>,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values, MAX_ASES, CONNECTIONS_MAX, NoopRawMutex>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    if let Some(peer) = bonded_peer {
        let params = AdvertisementParameters {
            timeout: Some(DIRECTED_RECONNECT_TIMEOUT),
            ..Default::default()
        };
        let advertiser = peripheral
            .advertise(&params, Advertisement::ConnectableNonscannableDirected { peer })
            .await?;
        defmt::info!("directed advertising to saved peer: {}", peer);
        match advertiser.accept().await {
            Ok(conn) => return Ok(conn.with_attribute_server(&server.server)?),
            Err(Error::Timeout) => defmt::info!("saved peer did not reconnect; advertising to all peers"),
            Err(error) => defmt::warn!("directed advertising failed: {}", Debug2Format(&error)),
        }
    }

    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data,
                scan_data: &[],
            },
        )
        .await?;
    defmt::info!("advertising");
    // `?` converts the attribute-server error into `BleHostError`; returning it directly would not.
    let conn = advertiser.accept().await?.with_attribute_server(&server.server)?;
    Ok(conn)
}
