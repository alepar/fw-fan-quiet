//! Focused integration checks retained across the legacy-controller deletion.

use super::device_loop::{ActuatorState, DeviceLoop, Gains, Hold, TickInput, W};
use super::guards::MaxRatchet;
use crate::sensors::ec::{EcReplica, ReconciliationObservation};

#[test]
fn actuator_mismatch_holds_then_verified_feedback_recovers() {
    let mut loop_ = DeviceLoop::<W>::new(Gains { kc: 1.0, ti_s: 30.0 });
    loop_.seed_candidates(30.0, 30.0, Some(30.0), 0.0);
    let input = |actuator| TickInput {
        dt_s: 1.0, t_star: 70.0, group_c: Some(68.0), draw: Some(20.0),
        floor: 10.0, max: 54.0, shadow_headroom: 10.0,
        shadow_fall_rate: 0.33, actuator, mode: super::device_loop::ThermalMode::Regulate,
        resumed: false, delta_tstar: 0.0, shadow_enabled: true,
    };
    let held = loop_.tick(input(ActuatorState::Mismatch));
    assert_eq!(held.hold, Hold::ActuatorMismatch);
    assert_eq!(held.cap, 30.0);
    let recovered = loop_.tick(input(ActuatorState::Verified));
    assert_ne!(recovered.hold, Hold::ActuatorMismatch);
    assert!(recovered.write_allowed);
}

#[test]
fn hot_guard_ratchet_lowers_to_floor_and_recovers_below_margin() {
    let mut ratchet = MaxRatchet::<W>::cpu(10.0, 54.0, 54.0, 90.0);
    for _ in 0..30 { ratchet.step(true, true, Some(92.0)); }
    let low = ratchet.step(true, true, Some(92.0));
    assert_eq!(low, 10.0);
    let recovering = ratchet.step(true, false, Some(84.0));
    assert!(recovering > low);
}

#[test]
fn reconciliation_three_strikes_and_three_matches_clear_safely() {
    let mut replica = EcReplica::new(1);
    replica.reset(Some(70.0), Some(70.0), Some(70.0));
    replica.tick(None);
    for _ in 0..3 {
        replica.score_reconciliation(ReconciliationObservation::Scored {
            max_matches: false, ma_matches: Some(true), socket_ma: 70.0,
        });
    }
    assert!(replica.ec_mismatch());
    for _ in 0..3 {
        replica.score_reconciliation(ReconciliationObservation::Scored {
            max_matches: true, ma_matches: Some(true), socket_ma: 70.0,
        });
    }
    assert!(!replica.ec_mismatch());
}
