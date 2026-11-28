#ifndef SAFETY_MODES_HYUNDAI_CANFD_H
#define SAFETY_MODES_HYUNDAI_CANFD_H

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/hyundai_common.h"

#define HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(bus) \
  {0x1CF, (bus), 8, .check_relay = false},  /* CRUISE_BUTTON */   \

#define HYUNDAI_CANFD_LKA_STEER_MSG_COMMON_TX_MSGS(a_can, e_can) \
  HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(e_can)                        \
  {0x50,  (a_can), 16, .check_relay = (a_can) == 0},  /* LKAS */      \
  {0x2A4, (a_can), 24, .check_relay = (a_can) == 0},  /* CAM_0x2A4 */ \

#define HYUNDAI_CANFD_LKA_STEER_MSG_ALT_COMMON_TX_MSGS(a_can, e_can) \
  HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(e_can)                        \
  {0x110, (a_can), 32, .check_relay = (a_can) == 0},  /* LKAS_ALT */  \
  {0x362, (a_can), 32, .check_relay = (a_can) == 0},  /* CAM_0x362 */ \

#define HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(e_can)  \
  {0x12A, (e_can), 16, .check_relay = (e_can) == 0},  /* LFA */            \
  {0x1E0, (e_can), 16, .check_relay = (e_can) == 0},  /* LFAHDA_CLUSTER */ \

#define HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(e_can, longitudinal) \
  {0x1A0, (e_can), 32, .check_relay = (longitudinal)},  /* SCC_CONTROL */ \

// *** Addresses checked in rx hook ***
// EV, ICE, HYBRID: ACCELERATOR (0x35), ACCELERATOR_BRAKE_ALT (0x100), ACCELERATOR_ALT (0x105)
#define HYUNDAI_CANFD_COMMON_RX_CHECKS(pt_bus)                                                                          \
  {.msg = {{0x35, (pt_bus), 32, 100U, .max_counter = 0xffU, .ignore_quality_flag = true},                  \
           {0x100, (pt_bus), 32, 100U, .max_counter = 0xffU, .ignore_quality_flag = true},                 \
           {0x105, (pt_bus), 32, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}}},               \
  {.msg = {{0x175, (pt_bus), 24, 50U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{0xa0, (pt_bus), 24, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{0xea, (pt_bus), 24, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

#define HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(pt_bus)                                                                                            \
  HYUNDAI_CANFD_COMMON_RX_CHECKS(pt_bus)                                                                                                       \
  {.msg = {{0x1cf, (pt_bus), 8, 50U, .ignore_checksum = true, .max_counter = 0xfU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

#define HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(pt_bus)                                                                                              \
  HYUNDAI_CANFD_COMMON_RX_CHECKS(pt_bus)                                                                                                         \
  {.msg = {{0x1aa, (pt_bus), 16, 50U, .ignore_checksum = true, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

// SCC_CONTROL (from ADAS unit or camera)
#define HYUNDAI_CANFD_SCC_ADDR_CHECK(scc_bus)                                                                            \
  {.msg = {{0x1a0, (scc_bus), 32, 50U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

static unsigned int hyundai_canfd_get_lka_addr(void) {
  return safety_setup.hyundai_canfd_lka_steer_msg_alt ? 0x110U : 0x50U;
}

// A host CCNC_0x161 copy older than this no longer replaces the stock camera frame
#define HYUNDAI_CANFD_CCNC_HOST_MAX_AGE_US 150000U

static uint8_t hyundai_canfd_get_counter(const CANPacket_t *msg) {
  uint8_t ret = 0;
  if (GET_LEN(msg) == 8U) {
    ret = msg->data[1] >> 4;
  } else {
    ret = msg->data[2];
  }
  return ret;
}

static uint32_t hyundai_canfd_get_checksum(const CANPacket_t *msg) {
  uint32_t chksum = GET_BYTES_LE(msg, 0, 2);
  return chksum;
}

static uint8_t hyundai_canfd_camera_mdps_state(uint8_t camera_state, uint8_t mdps_state) {
  const bool camera_operational = (camera_state == 1U) || (camera_state == 2U);
  const bool mdps_operational = (mdps_state == 1U) || (mdps_state == 2U);
  return (camera_operational && mdps_operational) ? camera_state : mdps_state;
}

static bool hyundai_canfd_pedal_resume_enabled(void) {
  return safety_setup.hyundai_longitudinal &&
         ((((uint32_t)safety_setup.alternative_experience) & ALT_EXP_PEDAL_RESUME) != 0U);
}

// ALT_EXP_PEDAL_RESUME: the driver's own pedal input seen here on the car bus, a brake release or
// an accelerator press or release, re-allows controls while cruise is available. The host decides whether to engage.
static void hyundai_canfd_pedal_resume(bool pedal_edge) {
  if (pedal_edge && safety.acc_main_on && !safety.hyundai_pedal_resume_blocked && !safety.brake_pressed) {
    safety.controls_allowed = true;
  }
}

// Without control authority the host may still show the openpilot cruise as ready on the cluster: the copy must be
// the held stock camera frame with only the display bits below changed, and those icons only hidden or gray (0 or 1),
// so warnings, sounds and every other camera signal stay stock and nothing looks engaged.
static bool hyundai_canfd_ccnc_display_only(const CANPacket_t *msg) {
  // CCNC_0x161 bits that may change, per byte: CHECKSUM (bytes 0-1, checked on its own), SETSPEED_HUD (byte 22 bits 5-7),
  // DISTANCE_CAR (byte 23 bits 5-7), DISTANCE gap bars (byte 24 bits 4-7), SETSPEED_SPEED (byte 25),
  // SETSPEED (byte 26 bits 0-3), NAV_ICON (byte 27 bits 4-7), LFA_ICON (byte 28 bits 0-3)
  static const uint8_t HYUNDAI_CANFD_CCNC_DISPLAY_BITS[32] = {
    0xFFU, 0xFFU, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0xE0U, 0xE0U, 0xF0U, 0xFFU, 0x0FU, 0xF0U, 0x0FU, 0x00U, 0x00U, 0x00U,
  };
  uint8_t diff = 0U;
  for (uint8_t i = 0U; i < 32U; i++) {
    diff |= (uint8_t)((msg->data[i] ^ safety.hyundai_ccnc_stock[i]) & (uint8_t)(~HYUNDAI_CANFD_CCNC_DISPLAY_BITS[i]));
  }
  const uint8_t setspeed_hud = (msg->data[22] >> 5U) & 0x7U;
  const uint8_t distance_car = (msg->data[23] >> 5U) & 0x7U;
  const uint8_t setspeed = msg->data[26] & 0xFU;
  const uint8_t nav_icon = (msg->data[27] >> 4U) & 0xFU;
  const uint8_t lfa_icon = msg->data[28] & 0xFU;
  const bool gray_icons = (setspeed_hud <= 1U) && (distance_car <= 1U) && (setspeed <= 1U) && (nav_icon <= 1U) &&
                          (lfa_icon <= 1U);
  return safety.hyundai_ccnc_stock_valid && (diff == 0U) && gray_icons;
}

static void hyundai_canfd_rx_hook(const CANPacket_t *msg) {
  const unsigned pt_bus = safety_setup.hyundai_canfd_lka_steer_msg ? 1U : 0U;
  const unsigned int scc_bus = safety_setup.hyundai_camera_scc ? 2U : pt_bus;

  if (safety_setup.hyundai_canfd_angle_steering && msg_matches(msg, 0x161U, 2U, 32U)) {
    for (uint8_t i = 0U; i < 32U; i++) {
      safety.hyundai_ccnc_stock[i] = msg->data[i];
    }
    safety.hyundai_ccnc_stock_valid = true;
  }

  if (safety_setup.hyundai_canfd_angle_steering && msg_matches(msg, 0xCBU, 2U)) {
    safety.hyundai_angle_stock_lfa_state = msg->data[3] >> 4U;
    for (uint8_t i = 7U; i < 24U; i++) {
      safety.hyundai_angle_stock_lfa[i - 7U] = msg->data[i];
    }
    safety.hyundai_angle_stock_lfa_valid = true;
  }

  // driver torque
  if (msg_matches(msg, 0xeaU, pt_bus)) {
    if (safety_setup.hyundai_canfd_angle_steering) {
      for (uint8_t i = 0U; i < 24U; i++) {
        safety.hyundai_angle_stock_mdps_prev[i] = safety.hyundai_angle_stock_mdps[i];
        safety.hyundai_angle_stock_mdps[i] = msg->data[i];
      }
      safety.hyundai_angle_stock_mdps_prev_valid = safety.hyundai_angle_stock_mdps_valid;
      safety.hyundai_angle_stock_mdps_valid = true;
    }

    int torque_driver_new = from_offset(GET_BYTES_LE(msg, 10, 2) & 0x1FFFU, 4095);
    update_sample(&safety.torque_driver, torque_driver_new);

    if (safety_setup.hyundai_canfd_angle_steering) {
      const uint16_t raw_angle = GET_BYTES_LE(msg, 16, 2);
      const bool valid_angle = (raw_angle != 0x7FFEU) && (raw_angle != 0x7FFFU);
      // Lv2 value 2 means "Fault Except MDPS". This is an external ADAS
      // source fault and does not invalidate the MDPS angle measurement or
      // actuator. Keep blocking MDPS-internal faults (bit 0) and every
      // temporarily-unavailable state (bit 2).
      const uint8_t lv2_fault = (msg->data[18] >> 4U) & 0x7U;
      const bool lv2_mdps_unavailable = ((lv2_fault & 0x1U) != 0U) || ((lv2_fault & 0x4U) != 0U);
      const bool mdps_fault = ((msg->data[3] & 0x7U) != 0U) || (((msg->data[6] >> 6U) & 0x3U) != 0U) ||
                              (((msg->data[15] >> 6U) & 0x3U) != 0U) || lv2_mdps_unavailable;
      if (valid_angle && !mdps_fault) {
        update_sample(&safety.angle_meas, to_signed(raw_angle, 16));
      } else {
        safety.controls_allowed = false;
      }
      safety.hyundai_angle_mdps_fault = !valid_angle || mdps_fault;
    }
  }

  // cruise buttons
  const unsigned int button_addr = safety_setup.hyundai_canfd_alt_buttons ? 0x1aaU : 0x1cfU;
  if (msg_matches(msg, button_addr, pt_bus)) {
    bool main_button = false;
    bool lfa_button = false;
    uint8_t cruise_button = 0U;
    if (msg_matches(msg, 0x1cfU, pt_bus)) {
      cruise_button = msg->data[2] & 0x7U;
      main_button = GET_BIT(msg, 19U);
      lfa_button = GET_BIT(msg, 23U);
    } else {
      cruise_button = (msg->data[4] >> 4) & 0x7U;
      main_button = GET_BIT(msg, 34U);
      lfa_button = GET_BIT(msg, 39U);
    }
    const uint8_t cruise_button_prev = safety.cruise_button_prev;
    hyundai_common_cruise_buttons_check(cruise_button, main_button);
    if (hyundai_canfd_pedal_resume_enabled()) {
      if (cruise_button == HYUNDAI_BTN_CANCEL) {
        safety.hyundai_pedal_resume_blocked = true;
      } else if ((cruise_button != cruise_button_prev) &&
                 ((cruise_button_prev == HYUNDAI_BTN_SET) || (cruise_button_prev == HYUNDAI_BTN_RESUME))) {
        safety.hyundai_pedal_resume_blocked = false;
      } else {
      }
    }

    // the LFA button is a driver engagement for always-on lateral
    if (lfa_button && always_on_lateral_enabled()) {
      safety.lateral_engaged = true;
    }
  }

  // gas press, different for EV, hybrid, and ICE models
  bool gas_msg = false;
  if (msg_matches(msg, 0x35U, pt_bus) && safety_setup.hyundai_ev_gas_signal) {
    safety.gas_pressed = msg->data[5] != 0U;
    gas_msg = true;
  }
  if (msg_matches(msg, 0x105U, pt_bus) && safety_setup.hyundai_hybrid_gas_signal) {
    safety.gas_pressed = ((msg->data[12] >> 7) | msg->data[13] | (msg->data[14] & 1U)) != 0U;
    gas_msg = true;
  }
  if (msg_matches(msg, 0x100U, pt_bus) && !safety_setup.hyundai_ev_gas_signal && !safety_setup.hyundai_hybrid_gas_signal) {
    safety.gas_pressed = GET_BIT(msg, 176U);
    gas_msg = true;
  }
  if (gas_msg && hyundai_canfd_pedal_resume_enabled()) {
    // accelerator: the moment it is pressed or released
    hyundai_canfd_pedal_resume(safety.gas_pressed_prev != safety.gas_pressed);
  }

  // brake press
  if (msg_matches(msg, 0x175U, pt_bus)) {
    safety.brake_pressed = GET_BIT(msg, 81U);
    if (hyundai_canfd_pedal_resume_enabled()) {
      // TCS ACCEnable: 0 means the car allows cruise (CarState.cruiseState.available)
      safety.acc_main_on = ((msg->data[8] >> 2) & 0x3U) == 0U;
      if (!safety.acc_main_on) {
        safety.controls_allowed = false;
      }
      // brake: the moment it is released
      hyundai_canfd_pedal_resume(safety.brake_pressed_prev && !safety.brake_pressed);
    }
  }

  // vehicle moving
  if (msg_matches(msg, 0xa0U, pt_bus)) {
    uint32_t fl = (GET_BYTES_LE(msg, 8, 2)) & 0x3FFFU;
    uint32_t fr = (GET_BYTES_LE(msg, 10, 2)) & 0x3FFFU;
    uint32_t rl = (GET_BYTES_LE(msg, 12, 2)) & 0x3FFFU;
    uint32_t rr = (GET_BYTES_LE(msg, 14, 2)) & 0x3FFFU;
    safety.vehicle_moving = (fl > HYUNDAI_STANDSTILL_THRSLD) || (fr > HYUNDAI_STANDSTILL_THRSLD) ||
                     (rl > HYUNDAI_STANDSTILL_THRSLD) || (rr > HYUNDAI_STANDSTILL_THRSLD);
    if (safety_setup.hyundai_canfd_angle_steering) {
      const float speed_kph = ((float)(fl + fr + rl + rr) / 4.0f) * 0.03125f;
      UPDATE_VEHICLE_SPEED(speed_kph * KPH_TO_MS);
    }
  }

  // cruise state
  if (!safety_setup.hyundai_longitudinal && msg_matches(msg, 0x1a0U, scc_bus)) {
    // 1=enabled, 2=driver override
    uint8_t cruise_status = (msg->data[8] >> 4) & 0x7U;
    bool cruise_engaged = (cruise_status == 1U) || (cruise_status == 2U);
    hyundai_common_cruise_state_check(cruise_engaged);
  }
}

static bool hyundai_canfd_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits HYUNDAI_CANFD_STEERING_LIMITS = {
    .max_torque = 270,
    .max_rt_delta = 112,
    .max_rate_up = 2,
    .max_rate_down = 3,
    .driver_torque_allowance = 250,
    .driver_torque_multiplier = 2,
    .type = TorqueDriverLimited,

    // the EPS faults when the steering angle is above a certain threshold for too long. to prevent this,
    // we allow setting torque actuation bit to 0 while maintaining the requested torque value for two consecutive frames
    .min_valid_request_frames = 89,
    .max_invalid_request_frames = 2,
    .min_valid_request_rt_interval = 810000,  // 810ms; a ~10% buffer on cutting every 90 frames
    .has_steer_req_tolerance = true,
  };

  bool tx = true;

  // Active angle commands are bounded by lateral accel and jerk (vehicle model) instead of a fixed
  // angle and speed table. The absolute 175 degree limit and the 10 degree measured error stay.
  const AngleSteeringLimits HYUNDAI_CANFD_ANGLE_LIMITS = {
    .max_angle = 1750,
    .angle_deg_to_can = 10.0f,
    .frequency = 100U,
  };

  // Used for every CAN-FD angle steering car. The values come from the first car measured with this mode
  // (IONIQ 9): slip factor from calc_slip_factor() with its CarParams, steer ratio and wheelbase from its
  // CarSpecs. sunnypilot uses the same values.
  const AngleSteeringParams HYUNDAI_CANFD_ANGLE_PARAMS = {
    .slip_factor = -0.0005358728290930592f,
    .steer_ratio = 16.02f,
    .wheelbase = 3.13f,
  };

  if (safety_setup.hyundai_canfd_angle_steering && msg_matches(msg, 0xCBU, 0U)) {
    const int desired_angle = to_signed(GET_BYTES_LE(msg, 4, 2) & 0x3FFFU, 14);
    const uint8_t state = msg->data[3] >> 4U;
    const bool steer_control_enabled = state == 2U;
    const uint8_t gain = msg->data[6];
    const uint8_t counter = msg->data[2];
    bool stock_payload_matches = safety.hyundai_angle_stock_lfa_valid && ((msg->data[3] & 0xFU) == 0U);
    for (uint8_t i = 7U; i < 24U; i++) {
      stock_payload_matches = stock_payload_matches && (msg->data[i] == safety.hyundai_angle_stock_lfa[i - 7U]);
    }

    // Active commands may only reduce the MDPS torque ceiling from its stock
    // maximum. Zero has no steering authority and is invalid while active;
    // values above the physical 1.0/250 encoding remain blocked.
    const bool state_violation = steer_control_enabled ? ((gain == 0U) || (gain > 250U)) :
                                                         ((state != 1U) || (gain != 0U));
    // without controls_allowed (always-on lateral), an MDPS fault blocks active control until it clears
    const bool lateral_mdps_fault = steer_control_enabled && !safety.controls_allowed && safety.hyundai_angle_mdps_fault;
    const bool vm_rt_checked = lateral_controls_allowed() && steer_control_enabled;
    // Driver torque does not block active frames: the controller keeps steering and lowers the MDPS
    // torque ceiling (gain) while the driver steers.
    const bool angle_violation = steer_angle_cmd_checks_vm(desired_angle, steer_control_enabled, HYUNDAI_CANFD_ANGLE_LIMITS,
                                                           HYUNDAI_CANFD_ANGLE_PARAMS) ||
                                 safety_max_limit_check(desired_angle, HYUNDAI_CANFD_ANGLE_LIMITS.max_angle,
                                                       -HYUNDAI_CANFD_ANGLE_LIMITS.max_angle);
    const bool measured_error = steer_control_enabled &&
                                safety_max_limit_check(desired_angle, safety.angle_meas.max + 100, safety.angle_meas.min - 100);
    // While steering, steer_angle_cmd_checks_vm already counts this frame for the real time rate limit.
    bool frequency_violation = false;
    if (!vm_rt_checked) {
      frequency_violation = rt_angle_rate_limit_check(HYUNDAI_CANFD_ANGLE_LIMITS);
    }
    const bool checksum_violation = hyundai_canfd_get_checksum(msg) != hyundai_common_canfd_compute_checksum(msg);
    const bool counter_violation = safety.hyundai_angle_tx_counter_valid &&
                                   (counter != ((safety.hyundai_angle_tx_counter + 1U) & 0xFFU));
    if (state_violation || lateral_mdps_fault || angle_violation || measured_error ||
        frequency_violation || checksum_violation || counter_violation || !stock_payload_matches) {
      tx = false;
      // Protocol, measurement, and frequency failures happen outside the
      // generic angle helper. Do not let a rejected frame advance its rate
      // limiter state and authorize a later jump.
      safety.desired_angle_last = safety_clamp_int(safety.angle_meas.values[0],
                                                   -HYUNDAI_CANFD_ANGLE_LIMITS.max_angle,
                                                   HYUNDAI_CANFD_ANGLE_LIMITS.max_angle);
    }
    // Resynchronize after a valid-CRC counter jump so one rejected request
    // cannot permanently lock out a correctly incrementing controller.
    if (!checksum_violation) {
      safety.hyundai_angle_tx_counter = counter;
      safety.hyundai_angle_tx_counter_valid = true;
    }
  }

  if (safety_setup.hyundai_canfd_angle_steering && msg_matches(msg, 0xEAU, 2U)) {
    const uint8_t counter = msg->data[2];
    const bool checksum_violation = hyundai_canfd_get_checksum(msg) != hyundai_common_canfd_compute_checksum(msg);
    const bool counter_violation = safety.hyundai_angle_mdps_tx_counter_valid &&
                                   (counter != ((safety.hyundai_angle_mdps_tx_counter + 1U) & 0xFFU));

    bool current_payload_matches = safety.hyundai_angle_stock_mdps_valid;
    bool previous_payload_matches = safety.hyundai_angle_stock_mdps_prev_valid;
    for (uint8_t i = 3U; i < 24U; i++) {
      if ((i != 6U) && (i != 18U)) {
        current_payload_matches = current_payload_matches && (msg->data[i] == safety.hyundai_angle_stock_mdps[i]);
        previous_payload_matches = previous_payload_matches && (msg->data[i] == safety.hyundai_angle_stock_mdps_prev[i]);
      }
    }
    const uint8_t current_stock_fault = (safety.hyundai_angle_stock_mdps[18] >> 4U) & 0x7U;
    const uint8_t previous_stock_fault = (safety.hyundai_angle_stock_mdps_prev[18] >> 4U) & 0x7U;
    const uint8_t requested_fault = (msg->data[18] >> 4U) & 0x7U;
    // Reflect the CAM-side request only for operational Lv2 states. Preserve
    // the measured MDPS state otherwise, including fault or unavailable states.
    const uint8_t camera_state = safety.hyundai_angle_stock_lfa_state;
    const uint8_t expected_current_state = hyundai_canfd_camera_mdps_state(camera_state, safety.hyundai_angle_stock_mdps[18] & 0xFU);
    const uint8_t expected_previous_state = hyundai_canfd_camera_mdps_state(camera_state, safety.hyundai_angle_stock_mdps_prev[18] & 0xFU);
    const uint8_t sent_mdps_state = msg->data[18] & 0xFU;
    current_payload_matches = current_payload_matches &&
                              (sent_mdps_state == expected_current_state) &&
                              ((msg->data[6] & 0xF0U) == (safety.hyundai_angle_stock_mdps[6] & 0xF0U)) &&
                              (requested_fault == (current_stock_fault == 0U ? 4U : current_stock_fault)) &&
                              ((msg->data[18] & 0x80U) == (safety.hyundai_angle_stock_mdps[18] & 0x80U));
    previous_payload_matches = previous_payload_matches &&
                               (sent_mdps_state == expected_previous_state) &&
                               ((msg->data[6] & 0xF0U) == (safety.hyundai_angle_stock_mdps_prev[6] & 0xF0U)) &&
                               (requested_fault == (previous_stock_fault == 0U ? 4U : previous_stock_fault)) &&
                               ((msg->data[18] & 0x80U) == (safety.hyundai_angle_stock_mdps_prev[18] & 0x80U));
    const bool payload_violation = ((msg->data[6] & 0xFU) != 0x4U) ||
                                   !safety.hyundai_angle_stock_lfa_valid ||
                                   (!current_payload_matches && !previous_payload_matches);

    if (checksum_violation || counter_violation || payload_violation) {
      tx = false;
    }
    if (!checksum_violation) {
      safety.hyundai_angle_mdps_tx_counter = counter;
      safety.hyundai_angle_mdps_tx_counter_valid = true;
    }
  }

  // CCNC_0x161 (cluster): the host copy of a stock camera frame goes out only in place of the stock frame
  // that the forwarding hook held back for it, once, while lateral control is allowed, or without control
  // when it changes only the cruise display (hyundai_canfd_ccnc_display_only).
  if (safety_setup.hyundai_canfd_angle_steering && msg_matches(msg, 0x161U, 0U)) {
    const bool display_only = hyundai_canfd_ccnc_display_only(msg);
    const bool valid_copy = (lateral_controls_allowed() || display_only) &&
                            (hyundai_canfd_get_checksum(msg) == hyundai_common_canfd_compute_checksum(msg));
    if (valid_copy) {
      safety.hyundai_ccnc_host_seen = true;
      safety.hyundai_ccnc_host_ts = safety_ts;
      safety.hyundai_ccnc_host_display = display_only;
    }
    if (!valid_copy || !safety.hyundai_ccnc_stock_pending) {
      tx = false;
    } else {
      safety.hyundai_ccnc_stock_pending = false;
    }
  }

  // steering
  const unsigned int steer_addr = (safety_setup.hyundai_canfd_lka_steer_msg && !safety_setup.hyundai_longitudinal) ? hyundai_canfd_get_lka_addr() : 0x12aU;
  if (msg->addr == steer_addr) {
    int desired_torque = from_offset((GET_BYTES_LE(msg, 5, 2) >> 1U) & 0x7FFU, 1024);
    bool steer_req = GET_BIT(msg, 52U);

    if (steer_torque_cmd_checks(desired_torque, steer_req, HYUNDAI_CANFD_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // cruise buttons check
  if (msg->addr == 0x1cfU) {
    uint8_t button = msg->data[2] & 0x7U;
    bool is_cancel = (button == HYUNDAI_BTN_CANCEL);
    bool is_resume = (button == HYUNDAI_BTN_RESUME);

    bool allowed = (is_cancel && safety.cruise_engaged_prev) || (is_resume && safety.controls_allowed);
    if (!allowed) {
      tx = false;
    }
  }

  // UDS: only tester present ("\x02\x3E\x80\x00\x00\x00\x00\x00") allowed on diagnostics address
  if ((safety_setup.hyundai_canfd_lka_steer_msg && (msg->addr == 0x730U)) || (!safety_setup.hyundai_camera_scc && (msg->addr == 0x7D0U))) {
    if (GET_BYTES_64_LE(msg, 0, 8) != 0x0000000000803E02ULL) {
      tx = false;
    }
  }

  // ACCEL: safety check
  if (msg->addr == 0x1a0U) {
    int desired_accel_raw = from_offset(GET_BYTES_LE(msg, 16, 2) & 0x7FFU, 1023);
    int desired_accel_val = from_offset(GET_BYTES_LE(msg, 17, 2) >> 4, 1023);

    bool violation = false;

    if (safety_setup.hyundai_longitudinal) {
      const bool raw_violation = longitudinal_accel_checks(desired_accel_raw, HYUNDAI_LONG_LIMITS);
      const bool val_violation = longitudinal_accel_checks(desired_accel_val, HYUNDAI_LONG_LIMITS);
      violation = raw_violation || val_violation;
    } else {
      // only used to cancel on here
      const uint8_t acc_mode = (msg->data[8] >> 4) & 0x7U;
      if (acc_mode != 4U) {
        violation = true;
      }

      if ((desired_accel_raw != 0) || (desired_accel_val != 0)) {
        violation = true;
      }
    }

    if (violation) {
      tx = false;
    }
  }

  return tx;
}

static bool hyundai_canfd_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;
  if (safety_setup.hyundai_canfd_angle_steering && (bus_num == 2) && (addr == 0x161)) {
    if (safety.hyundai_ccnc_stock_pending) {
      // the previous held-back stock frame was never replaced: forward stock until a new host copy arrives
      safety.hyundai_ccnc_host_seen = false;
    }
    const bool host_fresh = safety.hyundai_ccnc_host_seen &&
                            (safety_get_ts_elapsed(safety_ts, safety.hyundai_ccnc_host_ts) < HYUNDAI_CANFD_CCNC_HOST_MAX_AGE_US);
    block_msg = host_fresh && (lateral_controls_allowed() || safety.hyundai_ccnc_host_display);
    safety.hyundai_ccnc_stock_pending = block_msg;
  }
  return block_msg;
}

static safety_config hyundai_canfd_init(uint16_t param) {
  const uint16_t HYUNDAI_PARAM_CANFD_LKA_STEER_MSG_ALT = 128;
  const uint16_t HYUNDAI_PARAM_CANFD_ALT_BUTTONS = 32;
  const uint16_t HYUNDAI_PARAM_CANFD_ANGLE_STEERING = 1024;

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_COMMON_TX_MSGS(0, 1)
  };

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_ALT_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_ALT_COMMON_TX_MSGS(0, 1)
  };

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_LONG_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_COMMON_TX_MSGS(0, 1)
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(1)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(1, true)
    {0x51,  0, 32, .check_relay = false},  // ADRV_0x51
    {0x730, 1,  8, .check_relay = false},  // tester present for ADAS ECU disable
    {0x160, 1, 16, .check_relay = false},  // ADRV_0x160
    {0x1EA, 1, 32, .check_relay = false},  // ADRV_0x1ea
    {0x200, 1,  8, .check_relay = false},  // ADRV_0x200
    {0x345, 1,  8, .check_relay = false},  // ADRV_0x345
    {0x1DA, 1, 32, .check_relay = false},  // ADRV_0x1da
  };

  static const CanMsg HYUNDAI_CANFD_LFA_STEERING_TX_MSGS[] = {
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2)
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, false)
  };

  static const CanMsg HYUNDAI_CANFD_ANGLE_STEERING_TX_MSGS[] = {
    {0xCB, 0, 24, .check_relay = true},    // LFA_ALT
    {0xEA, 2, 24, .check_relay = true},    // camera MDPS mirror
    {0x161, 0, 32, .check_relay = false},  // CCNC_0x161 copy, see hyundai_canfd_fwd_hook
  };

  static const CanMsg HYUNDAI_CANFD_ANGLE_STEERING_LONG_TX_MSGS[] = {
    {0xCB,  0, 24, .check_relay = true},   // LFA_ALT
    {0xEA,  2, 24, .check_relay = true},   // camera MDPS mirror
    {0x1A0, 0, 32, .check_relay = true},   // SCC_CONTROL
    {0x161, 0, 32, .check_relay = false},  // CCNC_0x161 copy, see hyundai_canfd_fwd_hook
  };

  // ADRV_0x160 is checked for radar liveness
  static const CanMsg HYUNDAI_CANFD_LFA_STEERING_LONG_TX_MSGS[] = {
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2)
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, true)
    {0x160, 0, 16, .check_relay = true},  // ADRV_0x160
    {0x7D0, 0, 8, .check_relay = false},  // tester present for radar ECU disable
  };

  // ADRV_0x160 is checked for relay malfunction
#define HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(longitudinal) \
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2) \
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0) \
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, (longitudinal)) \
    {0x160, 0, 16, .check_relay = (longitudinal)}, /* ADRV_0x160 */ \

  hyundai_common_init(param);

  safety_setup.hyundai_canfd_alt_buttons = GET_FLAG(param, HYUNDAI_PARAM_CANFD_ALT_BUTTONS);
  safety_setup.hyundai_canfd_lka_steer_msg_alt = GET_FLAG(param, HYUNDAI_PARAM_CANFD_LKA_STEER_MSG_ALT);
  safety_setup.hyundai_canfd_angle_steering = GET_FLAG(param, HYUNDAI_PARAM_CANFD_ANGLE_STEERING);

  if (safety_setup.hyundai_canfd_angle_steering &&
      (!safety_setup.hyundai_camera_scc || safety_setup.hyundai_canfd_lka_steer_msg || safety_setup.hyundai_canfd_alt_buttons ||
       !safety_setup.hyundai_ev_gas_signal)) {
    return (safety_config){NULL, 0, NULL, 0, true};  // NOLINT(readability/braces)
  }

  safety_config ret;
  if (safety_setup.hyundai_canfd_angle_steering) {
    static const RxCheck hyundai_canfd_angle_rx_checks[] = {
      HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
      HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
      {.msg = {{0xCB, 2, 24, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    };
    // openpilot longitudinal also reads the stock camera cluster frame CCNC_0x161 (20 Hz), the reference for
    // host copies made without control authority (hyundai_canfd_ccnc_display_only)
    static const RxCheck hyundai_canfd_angle_long_rx_checks[] = {
      HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
      HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
      {.msg = {{0xCB, 2, 24, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},
      {.msg = {{0x161, 2, 32, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    };
    if (safety_setup.hyundai_longitudinal) {
      ret = BUILD_SAFETY_CFG(hyundai_canfd_angle_long_rx_checks, HYUNDAI_CANFD_ANGLE_STEERING_LONG_TX_MSGS);
    } else {
      ret = BUILD_SAFETY_CFG(hyundai_canfd_angle_rx_checks, HYUNDAI_CANFD_ANGLE_STEERING_TX_MSGS);
    }
  } else if (safety_setup.hyundai_longitudinal) {
    if (safety_setup.hyundai_canfd_lka_steer_msg) {
      static const RxCheck hyundai_canfd_lka_steer_msg_long_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(1)
      };

      ret = BUILD_SAFETY_CFG(hyundai_canfd_lka_steer_msg_long_rx_checks, HYUNDAI_CANFD_LKA_STEER_MSG_LONG_TX_MSGS);
    } else {
      // Longitudinal checks for LFA steering
      static const RxCheck hyundai_canfd_long_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
      };

      static const RxCheck hyundai_canfd_alt_buttons_long_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(0)
      };

      static const CanMsg hyundai_canfd_lfa_steering_camera_scc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(true)
      };

      if (safety_setup.hyundai_canfd_alt_buttons) {
        SET_RX_CHECKS(hyundai_canfd_alt_buttons_long_rx_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_canfd_long_rx_checks, ret);
      }

      if (safety_setup.hyundai_camera_scc) {
        SET_TX_MSGS(hyundai_canfd_lfa_steering_camera_scc_tx_msgs, ret);
      } else {
        SET_TX_MSGS(HYUNDAI_CANFD_LFA_STEERING_LONG_TX_MSGS, ret);
      }
    }
  } else {
    if (safety_setup.hyundai_canfd_lka_steer_msg) {
      // *** LKA steering checks ***
      // E-CAN is on bus 1, SCC messages are sent on cars with ADRV ECU.
      // Does not use the alt buttons message
      static const RxCheck hyundai_canfd_lka_steer_msg_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(1)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(1)
      };

      SET_RX_CHECKS(hyundai_canfd_lka_steer_msg_rx_checks, ret);
      if (safety_setup.hyundai_canfd_lka_steer_msg_alt) {
        SET_TX_MSGS(HYUNDAI_CANFD_LKA_STEER_MSG_ALT_TX_MSGS, ret);
      } else {
        SET_TX_MSGS(HYUNDAI_CANFD_LKA_STEER_MSG_TX_MSGS, ret);
      }
    } else if (!safety_setup.hyundai_camera_scc) {
      // Radar sends SCC messages on these cars instead of camera
      static const RxCheck hyundai_canfd_radar_scc_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(0)
      };

      static const RxCheck hyundai_canfd_alt_buttons_radar_scc_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(0)
      };

      SET_TX_MSGS(HYUNDAI_CANFD_LFA_STEERING_TX_MSGS, ret);

      if (safety_setup.hyundai_canfd_alt_buttons) {
        SET_RX_CHECKS(hyundai_canfd_alt_buttons_radar_scc_rx_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_canfd_radar_scc_rx_checks, ret);
      }
    } else {
      // *** LFA steering checks ***
      // Camera sends SCC messages on LFA steering cars.
      // Both button messages exist on some platforms, so we ensure we track the correct one using flag
      static const RxCheck hyundai_canfd_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
      };

      static const RxCheck hyundai_canfd_alt_buttons_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
      };

      static const CanMsg hyundai_canfd_lfa_steering_camera_scc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(false)
      };

      SET_TX_MSGS(hyundai_canfd_lfa_steering_camera_scc_tx_msgs, ret);

      if (safety_setup.hyundai_canfd_alt_buttons) {
        SET_RX_CHECKS(hyundai_canfd_alt_buttons_rx_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_canfd_rx_checks, ret);
      }
    }
  }

  return ret;
}

static const safety_hooks hyundai_canfd_hooks = {
  .init = hyundai_canfd_init,
  .rx = hyundai_canfd_rx_hook,
  .tx = hyundai_canfd_tx_hook,
  .fwd = hyundai_canfd_fwd_hook,
  .get_counter = hyundai_canfd_get_counter,
  .get_checksum = hyundai_canfd_get_checksum,
  .compute_checksum = hyundai_common_canfd_compute_checksum,
};

#endif  // SAFETY_MODES_HYUNDAI_CANFD_H
