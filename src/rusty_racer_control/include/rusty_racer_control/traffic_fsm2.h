// Copyright 2025 Rusty Racer Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file traffic_fsm2.h
 * @brief Traffic sign finite state machine - header-only implementation
 * @author zx
 * @date 2025-12
 *
 * Implements a three-layer FSM for traffic sign decision-making:
 *   Layer 0: Start gate (locked until lever up + stop sign gone)
 *   Layer 1: Speed mode (Default / Speed30 / Highway)
 *   Layer 2: Action triggers (Stop hold, Yield slow)
 *
 * All types and functions are defined inline for header-only usage.
 */

#ifndef RUSTY_RACER_CONTROL__TRAFFIC_FSM2_H_
#define RUSTY_RACER_CONTROL__TRAFFIC_FSM2_H_

#include <algorithm>
#include <cstdint>
#include <vector>
#include <string>
#include <type_traits>

// ═══════════════════════════════════════════════════════════════════════════════
//  Enums
// ═══════════════════════════════════════════════════════════════════════════════

enum class SignType: int
{
  Unknown = 6,
  Speed30Start = 0,
  Speed30End = 1,
  HighwayStart = 2,
  HighwayEnd = 3,
  Stop = 4,
  YieldSlow = 5
};

inline SignType signIdToType(int id)
{
  switch (id) {
    case 0: return SignType::HighwayEnd;      // end_express_way
    case 1: return SignType::Speed30End;      // end_zone_speed_limit
    case 2: return SignType::HighwayStart;    // start_express_way
    case 3: return SignType::Speed30Start;    // start_zone_speed_limit
    case 4: return SignType::Stop;            // stop_sign
    case 5: return SignType::YieldSlow;       // yield
    default: return SignType::Unknown;
  }
}

// String sign_id mapping (matches TrafficSign.msg labels)
inline SignType signIdToType(const std::string & id)
{
  if (id == "stop_sign") {return SignType::Stop;}
  if (id == "start_zone_speed_limit") {return SignType::Speed30Start;}
  if (id == "end_zone_speed_limit") {return SignType::Speed30End;}
  if (id == "start_express_way") {return SignType::HighwayStart;}
  if (id == "end_express_way") {return SignType::HighwayEnd;}
  if (id == "yield") {return SignType::YieldSlow;}
  return SignType::Unknown;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Data structs
// ═══════════════════════════════════════════════════════════════════════════════

struct CamDetection
{
  SignType type = SignType::Unknown;
  float distance_m = 0.0f;
  float confidence = 0.0f;
  bool valid = false;
};

struct CamFrame
{
  std::vector < CamDetection > dets;
};

struct DecisionOut
{
  float v_ref_mps = 0.0f;
  bool must_stop = false;
};

struct TrafficParams
{
  float conf_th = 0.0f;

  float v_default = 1.5f;
  float v_speed30 = 1.2f;
  float v_highway = 1.8f;
  float v_yield = 0.5f;

  float d_stop_trigger = 1.0f;
  float d_yield_trigger = 1.0f;
  float d_release = 3.0f;

  uint32_t stop_hold_ms = 3000;

  int on_count = 3;
  int off_count = 3;

  // float gate_clear_dist_m = 0.20f;
  // int gate_on_count = 3;
  // int gate_off_count = 3;
  int start_off_count = 6;
  float start_stop_lock_dist_m = 0.30f;

  // One-shot block distance: within 1.0m, do NOT re-trigger same sign
  float same_sign_block_dist_m = 1.0f;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  toCamFrame  (template so it works with both ROS msg and MockMsg)
// ═══════════════════════════════════════════════════════════════════════════════

// Dispatch helpers: resolve sign_id to SignType for string or integer types.
// Uses overload resolution to avoid if constexpr (unsupported by older uncrustify).
inline SignType resolveSignId(const std::string & id)
{
  return signIdToType(id);
}

template < typename T >
inline SignType resolveSignId(const T & id)
{
  return signIdToType(static_cast < int > (id));
}

template < typename Msg >
inline CamFrame toCamFrame(const Msg & msg)
{
  CamFrame frame;
  SignType t = resolveSignId(msg.sign_id);

  if (t == SignType::Unknown) {return frame;}
  if (msg.distance <= 0.0f) {return frame;}

  CamDetection det;
  det.type = t;
  det.distance_m = msg.distance;
  det.confidence = 1.0f;
  det.valid = true;
  frame.dets.push_back(det);
  return frame;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Internal enums
// ═══════════════════════════════════════════════════════════════════════════════

enum class SpeedMode: int
{
  Default = 0,
  Speed30 = 1,
  Highway = 2
};

enum class ActionState: int
{
  None = 0,
  StopHold = 1,
  YieldSlow = 2
};

// ═══════════════════════════════════════════════════════════════════════════════
//  TrafficFSM
// ═══════════════════════════════════════════════════════════════════════════════

class TrafficFSM {
public:
  TrafficFSM() {
    reset();
  }

  void reset()
  {
    gate_locked_ = true;
    gate_lever_up_ = false;
    stop_gone_count_ = 0;

    speed_mode_ = SpeedMode::Default;

    mode_s30_on_ = 0;
    mode_s30_off_ = 0;
    mode_s30_end_on_ = 0;
    mode_s30_end_off_ = 0;
    mode_hw_on_ = 0;
    mode_hw_off_ = 0;
    mode_hw_end_on_ = 0;
    mode_hw_end_off_ = 0;

    action_state_ = ActionState::None;
    action_elapsed_ms_ = 0;

    stop_on_count_ = 0;
    yield_on_count_ = 0;

    stop_cooldown_ = false;
    yield_cooldown_ = false;
    stop_last_dist_ = 0.0f;
    yield_last_dist_ = 0.0f;

    // ── One-shot arming flags ──
    stop_armed_ = true;
    yield_armed_ = true;
  }

  DecisionOut step(
    const CamFrame & cam, uint32_t dt_ms, const TrafficParams & p)
  {
    DecisionOut out;

    // ── Extract detections ───────────────────────────────────────────
    bool has_stop = false;
    float stop_dist = 999.0f;
    bool has_yield = false;
    float yield_dist = 999.0f;
    bool has_s30_start = false;
    bool has_s30_end = false;
    bool has_hw_start = false;
    bool has_hw_end = false;

    for (const auto & d : cam.dets) {
      if (!d.valid) {continue;}
      if (d.confidence < p.conf_th) {continue;}
      switch (d.type) {
        case SignType::Stop:
          has_stop = true;
          stop_dist = d.distance_m;
          break;
        case SignType::YieldSlow:
          has_yield = true;
          yield_dist = d.distance_m;
          break;
        case SignType::Speed30Start: has_s30_start = true; break;
        case SignType::Speed30End:   has_s30_end = true; break;
        case SignType::HighwayStart: has_hw_start = true; break;
        case SignType::HighwayEnd:   has_hw_end = true; break;
        default: break;
      }
    }

    // ── Layer 0: Start gate ──────────────────────────────────────────
    if (gate_locked_) {
      if (!has_stop) {
        stop_gone_count_++;
      } else {
        stop_gone_count_ = 0;
      }

      if (stop_gone_count_ >= p.start_off_count) {
        gate_locked_ = false;
      }

      if (gate_locked_) {
        out.must_stop = true;
        out.v_ref_mps = 0.0f;
        return out;
      }
    }

    // ── Layer 1: Speed mode switching ────────────────────────────────

    // start_zone_speed_limit
    if (has_s30_start) {
      mode_s30_on_++;
      mode_s30_off_ = 0;
    } else {
      mode_s30_off_++;
      if (mode_s30_off_ >= p.off_count) {
        mode_s30_on_ = 0;
      }
    }
    if (mode_s30_on_ >= p.on_count && speed_mode_ != SpeedMode::Speed30) {
      speed_mode_ = SpeedMode::Speed30;
    }

    // end_zone_speed_limit
    if (has_s30_end) {
      mode_s30_end_on_++;
      mode_s30_end_off_ = 0;
    } else {
      mode_s30_end_off_++;
      if (mode_s30_end_off_ >= p.off_count) {
        mode_s30_end_on_ = 0;
      }
    }
    if (mode_s30_end_on_ >= p.on_count && speed_mode_ == SpeedMode::Speed30) {
      speed_mode_ = SpeedMode::Default;
      mode_s30_on_ = 0;
      mode_s30_end_on_ = 0;
    }

    // start_express_way
    if (has_hw_start) {
      mode_hw_on_++;
      mode_hw_off_ = 0;
    } else {
      mode_hw_off_++;
      if (mode_hw_off_ >= p.off_count) {
        mode_hw_on_ = 0;
      }
    }
    if (mode_hw_on_ >= p.on_count && speed_mode_ != SpeedMode::Highway) {
      speed_mode_ = SpeedMode::Highway;
    }

    // end_express_way
    if (has_hw_end) {
      mode_hw_end_on_++;
      mode_hw_end_off_ = 0;
    } else {
      mode_hw_end_off_++;
      if (mode_hw_end_off_ >= p.off_count) {
        mode_hw_end_on_ = 0;
      }
    }
    if (mode_hw_end_on_ >= p.on_count && speed_mode_ == SpeedMode::Highway) {
      speed_mode_ = SpeedMode::Default;
      mode_hw_on_ = 0;
      mode_hw_end_on_ = 0;
    }

    // Base speed from mode
    float v_mode = p.v_default;
    switch (speed_mode_) {
      case SpeedMode::Speed30: v_mode = p.v_speed30; break;
      case SpeedMode::Highway: v_mode = p.v_highway; break;
      default: break;
    }

    // ── Layer 2: Action triggers ─────────────────────────────────────

    // Handle ongoing StopHold
    if (action_state_ == ActionState::StopHold) {
      action_elapsed_ms_ += dt_ms;
      if (action_elapsed_ms_ >= p.stop_hold_ms) {
        action_state_ = ActionState::None;
        action_elapsed_ms_ = 0;
        stop_cooldown_ = true;
        stop_on_count_ = 0;
      } else {
        out.must_stop = true;
        out.v_ref_mps = 0.0f;
        return out;
      }
    }

    // Handle ongoing YieldSlow
    if (action_state_ == ActionState::YieldSlow) {
      if (has_yield && yield_dist <= p.d_yield_trigger) {
        out.must_stop = false;
        out.v_ref_mps = p.v_yield;
        return out;
      } else {
        action_state_ = ActionState::None;
        yield_cooldown_ = true;
        yield_on_count_ = 0;
      }
    }

    // ── One-shot re-arm: leaving 1m range (or not seen) re-enables triggers ──
    if (!has_stop || stop_dist > p.same_sign_block_dist_m) {
      stop_armed_ = true;
    }
    if (!has_yield || yield_dist > p.same_sign_block_dist_m) {
      yield_armed_ = true;
    }

    // Cooldown management(kept for minimal diff; no longer relied upon)
    if (stop_cooldown_) {
      if (has_stop && stop_dist >= p.d_release) {
        stop_cooldown_ = false;
      }
      if (stop_cooldown_) {
        stop_on_count_ = 0;
      }
    }

    if (yield_cooldown_) {
      if (has_yield && yield_dist >= p.d_release) {
        yield_cooldown_ = false;
      }
      if (yield_cooldown_) {
        yield_on_count_ = 0;
      }
    }

    // New Stop trigger (debounced + one-shot)
    if (stop_armed_ && !stop_cooldown_) {
      if (has_stop && stop_dist <= p.d_stop_trigger) {
        stop_on_count_++;
      } else {
        stop_on_count_ = 0;
      }
      if (stop_on_count_ >= p.on_count) {
        action_state_ = ActionState::StopHold;
        action_elapsed_ms_ = 0;
        out.must_stop = true;
        out.v_ref_mps = 0.0f;

        // one-shot: disarm until leaving 1m range
        stop_armed_ = false;
        stop_on_count_ = 0;

        return out;
      }
    } else {
      stop_on_count_ = 0;
    }

    // New Yield trigger (debounced + one-shot)
    if (yield_armed_ && !yield_cooldown_) {
      if (has_yield && yield_dist <= p.d_yield_trigger) {
        yield_on_count_++;
      } else {
        yield_on_count_ = 0;
      }
      if (yield_on_count_ >= p.on_count) {
        action_state_ = ActionState::YieldSlow;
        out.must_stop = false;
        out.v_ref_mps = p.v_yield;

        // one-shot: disarm until leaving 1m range
        yield_armed_ = false;
        yield_on_count_ = 0;

        return out;
      }
    } else {
      yield_on_count_ = 0;
    }

    // Default output
    out.must_stop = false;
    out.v_ref_mps = v_mode;
    return out;
  }

private:
  // Layer 0: Start gate
  bool gate_locked_ = true;
  bool gate_lever_up_ = false;
  int stop_gone_count_ = 0;

  // Layer 1: Speed mode
  SpeedMode speed_mode_ = SpeedMode::Default;

  int mode_s30_on_ = 0;
  int mode_s30_off_ = 0;
  int mode_s30_end_on_ = 0;
  int mode_s30_end_off_ = 0;
  int mode_hw_on_ = 0;
  int mode_hw_off_ = 0;
  int mode_hw_end_on_ = 0;
  int mode_hw_end_off_ = 0;

  // Layer 2: Actions
  ActionState action_state_ = ActionState::None;
  uint32_t action_elapsed_ms_ = 0;

  int stop_on_count_ = 0;
  int yield_on_count_ = 0;

  bool stop_cooldown_ = false;
  bool yield_cooldown_ = false;
  float stop_last_dist_ = 0.0f;
  float yield_last_dist_ = 0.0f;

  // One-shot arming flags
  bool stop_armed_ = true;
  bool yield_armed_ = true;
};

#endif  // RUSTY_RACER_CONTROL__TRAFFIC_FSM2_H_
