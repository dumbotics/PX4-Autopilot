#include "intercept.h"
#include "navigator.h"

#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>

Intercept::Intercept(Navigator *navigator) :
	MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_INTERCEPT),
	ModuleParams(navigator)
{
}

void Intercept::on_inactive()
{
	_phase = Phase::Idle;
	_phase_started = 0;
	_intercept_sp = {};
}

void Intercept::on_activation()
{
	if (!update_plan(true)) {
		_phase = Phase::Idle;
		return;
	}

	_phase = Phase::Entry;
	_phase_started = hrt_absolute_time();
	reset_mission_item_reached();
	publish_entry_setpoint();
}

void Intercept::on_active()
{
	if (!update_plan(false)) {
		return;
	}

	switch (_phase) {
	case Phase::Entry:
		publish_entry_setpoint();

		if (is_mission_item_reached_or_completed()) {
			reset_mission_item_reached();
			_phase = Phase::RunIn;
			_phase_started = hrt_absolute_time();
		}
		break;

	case Phase::RunIn:
		publish_runin_setpoint();

		if (is_mission_item_reached_or_completed()) {
			reset_mission_item_reached();

			if (PX4_ISFINITE(_egress_lat) && PX4_ISFINITE(_egress_lon)) {
				_phase = Phase::Egress;
			} else {
				_phase = Phase::Complete;
			}

			_phase_started = hrt_absolute_time();
		}
		break;

	case Phase::Egress:
		publish_egress_setpoint();

		if (is_mission_item_reached_or_completed()) {
			_phase = Phase::Complete;
			_phase_started = hrt_absolute_time();
		}
		break;

	case Phase::Complete:
	default:
		// For MVP: do nothing (caller can switch to RTL)
		break;
	}
}

bool Intercept::plan_valid() const
{
	return _intercept_sp.valid
		&& PX4_ISFINITE(_intercept_sp.lat)
		&& PX4_ISFINITE(_intercept_sp.lon)
		&& PX4_ISFINITE(_intercept_sp.alt_amsl)
		&& PX4_ISFINITE(_intercept_sp.approach_course)
		&& _intercept_sp.standoff_distance > 0.f;
}

bool Intercept::update_plan(bool force)
{
	if (!force && !_intercept_setpoint_sub.updated()) {
		return plan_valid();
	}

	_intercept_setpoint_sub.copy(&_intercept_sp);

	if (!plan_valid()) {
		return false;
	}

	_intercept_lat = _intercept_sp.lat;
	_intercept_lon = _intercept_sp.lon;
	_intercept_alt_amsl = _intercept_sp.alt_amsl;

	const float desired_speed = _intercept_sp.cruise_speed > 0.f ? _intercept_sp.cruise_speed : -1.f;
	_navigator->set_cruising_speed(desired_speed);

	compute_entry_egress();
	return true;
}

void Intercept::compute_entry_egress()
{
	const float course = _intercept_sp.approach_course;
	const float standoff = _intercept_sp.standoff_distance;

	waypoint_from_heading_and_distance(_intercept_lat, _intercept_lon,
					   course + M_PI_F, standoff,
					   &_entry_lat, &_entry_lon);
	_entry_alt_amsl = _intercept_alt_amsl;

	if (_intercept_sp.run_length > 0.f) {
		waypoint_from_heading_and_distance(_intercept_lat, _intercept_lon,
						   course, _intercept_sp.run_length,
						   &_egress_lat, &_egress_lon);
		_egress_alt_amsl = _intercept_alt_amsl;
	} else {
		_egress_lat = NAN;
		_egress_lon = NAN;
		_egress_alt_amsl = NAN;
	}
}

void Intercept::publish_entry_setpoint()
{
	mission_item_s item{};
	PositionYawSetpoint pos_yaw{};

	pos_yaw.lat = _entry_lat;
	pos_yaw.lon = _entry_lon;
	pos_yaw.alt = _entry_alt_amsl;
	pos_yaw.yaw = NAN;

	setMoveToPositionMissionItem(item, pos_yaw);

	item.acceptance_radius = _intercept_sp.acceptance_radius > 0.f ?
				 _intercept_sp.acceptance_radius : _navigator->get_acceptance_radius();

	_mission_item = item;

	const auto *gpos = _navigator->get_global_position();
	publish_triplet_from_item(item, gpos->lat, gpos->lon, gpos->alt);
}

void Intercept::publish_runin_setpoint()
{
	mission_item_s item{};
	PositionYawSetpoint pos_yaw{};

	pos_yaw.lat = _intercept_lat;
	pos_yaw.lon = _intercept_lon;
	pos_yaw.alt = _intercept_alt_amsl;
	pos_yaw.yaw = NAN;

	setMoveToPositionMissionItem(item, pos_yaw);

	item.acceptance_radius = _intercept_sp.acceptance_radius > 0.f ?
				 _intercept_sp.acceptance_radius : _navigator->get_acceptance_radius();

	_mission_item = item;

	publish_triplet_from_item(item, _entry_lat, _entry_lon, _entry_alt_amsl);
}

void Intercept::publish_egress_setpoint()
{
	mission_item_s item{};
	PositionYawSetpoint pos_yaw{};

	pos_yaw.lat = _egress_lat;
	pos_yaw.lon = _egress_lon;
	pos_yaw.alt = _egress_alt_amsl;
	pos_yaw.yaw = NAN;

	setMoveToPositionMissionItem(item, pos_yaw);

	item.acceptance_radius = _intercept_sp.acceptance_radius > 0.f ?
				 _intercept_sp.acceptance_radius : _navigator->get_acceptance_radius();

	_mission_item = item;

	publish_triplet_from_item(item, _intercept_lat, _intercept_lon, _intercept_alt_amsl);
}

void Intercept::publish_triplet_from_item(const mission_item_s &item,
					  double prev_lat, double prev_lon, float prev_alt_amsl)
{
	position_setpoint_triplet_s *triplet = _navigator->get_position_setpoint_triplet();

	triplet->previous = {};
	triplet->previous.valid = true;
	triplet->previous.lat = prev_lat;
	triplet->previous.lon = prev_lon;
	triplet->previous.alt = prev_alt_amsl;

	triplet->current = {};
	mission_item_to_position_setpoint(item, &triplet->current);

	triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}
