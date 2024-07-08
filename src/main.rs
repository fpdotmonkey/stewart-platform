//! Demonstrate setting outputs using a Beckhoff EK1100/EK1501 and modules.
//!
//! Run with e.g.
//!
//! Linux
//!
//! ```bash
//! RUST_LOG=debug cargo run --example ek1100 --release -- eth0
//! ```
//!
//! Windows
//!
//! ```ps
//! $env:RUST_LOG="debug" ; cargo run --example ek1100 --release -- '\Device\NPF_{FF0ACEE6-E8CD-48D5-A399-619CD2340465}'
//! ```

use std::{sync::Arc, time::Duration};

use env_logger::Env;
use ethercrab::{
    error::Error,
    std::{ethercat_now, tx_rx_task},
    Client, ClientConfig, PduStorage, Timeouts,
};
use ethercrab_wire::EtherCrabWireRead;
use tokio::time::MissedTickBehavior;

mod controller;

/// Maximum number of slaves that can be stored. This must be a power of 2 greater than 1.
const MAX_SLAVES: usize = 16;
/// Maximum PDU data payload size - set this to the max PDI size or higher.
const MAX_PDU_DATA: usize = PduStorage::element_size(1100);
/// Maximum number of EtherCAT frames that can be in flight at any one time.
const MAX_FRAMES: usize = 16;
/// Maximum total PDI length.
const PDI_LEN: usize = 64;

static PDU_STORAGE: PduStorage<MAX_FRAMES, MAX_PDU_DATA> = PduStorage::new();

#[derive(Debug, ethercrab_wire::EtherCrabWireRead)]
#[wire(bytes = 4)]
struct El3062Reading {
    #[wire(bits = 1)]
    underrange: bool,
    #[wire(bits = 1)]
    overrange: bool,
    #[wire(bits = 2)]
    limit1: u8,
    #[wire(bits = 2)]
    limit2: u8,
    #[wire(bits = 1)]
    error: bool,
    #[wire(pre_skip = 7, bits = 1)]
    tx_pdo_state: bool,
    #[wire(bits = 1)]
    tx_pdo_toggle: bool,
    #[wire(bits = 16)]
    value: u16,
}

struct SetpointUpdate {
    ready: bool,
    setpoint: f64,
}

#[tokio::main]
async fn main() -> Result<(), Error> {
    env_logger::Builder::from_env(Env::default().default_filter_or("info")).init();

    let interface: String = match std::env::args().nth(1) {
        Some(arg) => arg,
        None => {
            eprintln!("provide a network device as the first argument");
            return Err(Error::Internal);
        }
    };

    log::info!("Starting EK1100/EK1501 demo...");
    log::info!(
        "Ensure an EK1100 or EK1501 is the first slave, with any number of modules connected after"
    );
    log::info!("Run with RUST_LOG=ethercrab=debug or =trace for debug information");

    let (tx, rx, pdu_loop) = PDU_STORAGE.try_split().expect("can only split once");

    let client = Arc::new(Client::new(
        pdu_loop,
        Timeouts {
            wait_loop_delay: Duration::from_millis(2),
            mailbox_response: Duration::from_millis(1000),
            ..Default::default()
        },
        ClientConfig::default(),
    ));

    tokio::spawn(tx_rx_task(&interface, tx, rx).expect("spawn TX/RX task"));

    let mut group = client
        .init_single_group::<MAX_SLAVES, PDI_LEN>(ethercat_now)
        .await
        .expect("Init");

    log::info!("Discovered {} slaves", group.len());

    for slave in group.iter(&client) {
        if slave.name() == "EL3062" {
            slave.sdo_write(0x1c12, 0, 0u8).await?;
            slave.sdo_write(0x1c13, 0, 0u8).await?;

            slave.sdo_write(0x1c13, 1, 0x1a00u16).await?;
            slave.sdo_write(0x1c13, 2, 0x1a02u16).await?;
            slave.sdo_write(0x1c13, 0, 2u8).await?;
        }
    }

    let mut group = group.into_op(&client).await.expect("PRE-OP -> OP");

    for slave in group.iter(&client) {
        let (i, o) = slave.io_raw();

        log::info!(
            "-> Slave {:#06x} {} inputs: {} bytes, outputs: {} bytes",
            slave.configured_address(),
            slave.name(),
            i.len(),
            o.len()
        );
    }

    let mut tick_interval = tokio::time::interval(Duration::from_millis(10));
    tick_interval.set_missed_tick_behavior(MissedTickBehavior::Skip);

    let shutdown = Arc::new(std::sync::atomic::AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shutdown))
        .expect("Register hook");

    let initial_setpoint = 1.0;
    let setpoint: std::sync::Arc<std::sync::Mutex<SetpointUpdate>> =
        std::sync::Arc::new(std::sync::Mutex::new(SetpointUpdate {
            setpoint: initial_setpoint,
            ready: false,
        }));
    spawn_interactive_tty_channel(setpoint.clone());
    let mut controller = controller::CylinderPositionController::new(
        controller::ControlGains::P(1.0),
        initial_setpoint,
    );
    loop {
        // graceful shutdown on ^C
        if shutdown.load(std::sync::atomic::Ordering::Relaxed) {
            log::info!("Shutting down...");
            break;
        }
        group.tx_rx(&client).await.expect("TX/RX");

        if setpoint.lock().unwrap().ready {
            let new_setpoint = setpoint.lock().unwrap().setpoint;
            controller.new_setpoint(new_setpoint);
            setpoint.lock().unwrap().ready = false;
        };

        let mut measurement_signal: Option<f64> = None;
        if let Some(el3062) = group.iter(&client).find(|slave| slave.name() == "EL3062") {
            let (i, _) = el3062.io_raw();
            if let Ok(channel1) = El3062Reading::unpack_from_slice(&i[..4]) {
                measurement_signal = Some(channel1.value as f64 / u16::MAX as f64);
            }
        }

        let control_signal = if let Some(measurement_signal) = measurement_signal {
            controller.control_signal(measurement_signal)
        } else {
            0.0
        };
        if let Some(mut el2042) = group.iter(&client).find(|slave| slave.name() == "EL2042") {
            let (_, o) = el2042.io_raw_mut();
            let deadband_half_width = 0.01;
            o[0] = if control_signal > deadband_half_width {
                0b10
            } else if control_signal < -deadband_half_width {
                0b01
            } else {
                0b00
            };
        };

        tick_interval.tick().await;
    }

    let group = group.into_safe_op(&client).await.expect("OP -> SAFE-OP");
    let group = group.into_pre_op(&client).await.expect("SAFE-OP -> PRE-OP");
    let _group = group.into_init(&client).await.expect("PRE-OP -> INIT");

    Ok(())
}

fn spawn_interactive_tty_channel(setpoint: std::sync::Arc<std::sync::Mutex<SetpointUpdate>>) {
    std::thread::spawn(move || loop {
        let mut buffer = String::new();
        std::io::stdin().read_line(&mut buffer).unwrap();
        match buffer.trim().parse() {
            Ok(new_setpoint) => {
                *setpoint.lock().unwrap() = SetpointUpdate {
                    setpoint: new_setpoint,
                    ready: true,
                }
            }
            Err(_) => eprintln!("setpoint must be a real number"),
        }
    });
}
