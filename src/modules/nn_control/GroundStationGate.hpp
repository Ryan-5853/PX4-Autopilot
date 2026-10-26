/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <cmath>
#include <stdint.h>

/**
 * RC-supervised ground-station control gate.
 *
 * The ground station is never allowed to become the primary manual-control
 * source. It can only replace the four normalized manual axes while all of
 * these conditions hold:
 *
 * - the vehicle is armed;
 * - a fresh physical RC manual-control sample exists;
 * - the RC flight-mode switch is in the ground-station mode slot;
 * - a fresh ground-station command exists;
 * - the mode slot was selected after arming.
 *
 * Leaving the mode slot, disarming, losing RC, or losing the ground-station
 * stream immediately returns to RC. A command-stream interruption also
 * requires the pilot to leave the ground-station mode slot and select it
 * again, so a reconnected ground station cannot silently resume control.
 *
 * This class is dependency-free so the exact state machine can be exercised
 * by a host test.
 */
class GroundStationGate
{
public:
	enum class Source : uint8_t {
		None = 0,
		Rc,
		GroundStation,
	};

	struct Config {
		bool enabled{false};
		uint8_t mode_slot{6};
		uint64_t entry_delay_us{100000};
	};

	struct Input {
		uint64_t now{0};
		bool armed{false};
		bool rc_valid{false};
		bool mode_slot_valid{false};
		uint8_t mode_slot{0};
		bool external_mode_active{false};
		bool ground_station_valid{false};
	};

	Source update(const Config &config, const Input &input);

	Source source() const { return _source; }
	bool mode_selected() const { return _mode_selected; }

	void reset()
	{
		_last_armed = false;
		_entry_authorized = false;
		_mode_selected = false;
		_selected_since = 0;
		_source = Source::None;
	}

private:
	bool _last_armed{false};
	bool _entry_authorized{false};
	bool _mode_selected{false};
	uint64_t _selected_since{0};
	Source _source{Source::None};
};

inline GroundStationGate::Source GroundStationGate::update(const Config &config, const Input &input)
{
	const bool armed_rise = input.armed && !_last_armed;
	_last_armed = input.armed;

	const bool requested_mode = input.mode_slot_valid && input.mode_slot == config.mode_slot;
	_mode_selected = requested_mode && input.external_mode_active;

	if (!config.enabled || config.mode_slot < 1 || config.mode_slot > 6 || !input.rc_valid) {
		_entry_authorized = false;
		_selected_since = 0;
		_source = input.rc_valid ? Source::Rc : Source::None;
		return _source;
	}

	if (!input.armed) {
		_entry_authorized = false;
		_selected_since = 0;
		_source = Source::Rc;
		return _source;
	}

	if (!input.mode_slot_valid) {
		_entry_authorized = false;
		_selected_since = 0;
		_source = Source::Rc;
		return _source;
	}

	// Arming while already in the ground-station slot is not sufficient.
	// Leave and reselect the slot after the arming edge.
	if (armed_rise) {
		_entry_authorized = !requested_mode;
		_selected_since = 0;
		_source = Source::Rc;
		return _source;
	}

	// Selecting any non-ground-station slot arms the next transition into
	// mode 6. This makes manual mode the explicit RC fallback.
	if (!requested_mode) {
		_entry_authorized = true;
		_selected_since = 0;
		_source = Source::Rc;
		return _source;
	}

	// The physical RC switch can become visible one control cycle before
	// Commander publishes the matching external nav_state. Keep RC selected
	// and preserve the authorization acquired while the pilot was in manual;
	// otherwise that ordinary transient permanently disables mode 6 until the
	// pilot deselects and reselects it.
	if (!input.external_mode_active) {
		if (_source == Source::GroundStation) {
			_entry_authorized = false;
		}

		_selected_since = 0;
		_source = Source::Rc;
		return _source;
	}

	// A stream interruption while ground-station control is active is a
	// hard interruption, not a pause. The pilot must leave and reselect mode 6
	// after the stream returns.
	if (_source == Source::GroundStation && !input.ground_station_valid) {
		_entry_authorized = false;
		_selected_since = 0;
		_source = Source::Rc;
		return _source;
	}

	if (!_entry_authorized) {
		_selected_since = 0;
		_source = Source::Rc;
		return _source;
	}

	if (_selected_since == 0) {
		_selected_since = input.now;
	}

	const bool entry_delay_elapsed = (input.now >= _selected_since)
					 && (input.now - _selected_since >= config.entry_delay_us);

	if (input.ground_station_valid && entry_delay_elapsed) {
		_source = Source::GroundStation;

	} else {
		_source = Source::Rc;
	}

	return _source;
}
