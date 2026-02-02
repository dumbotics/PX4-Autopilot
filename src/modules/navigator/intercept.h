#pragma once

#include "mission_block.h"

#include <uORB/Subscription.hpp>
#include <uORB/topics/intercept_setpoint.h>
#include <px4_platform_common/module_params.h>

class Intercept : public MissionBlock, public ModuleParams
{
public:
	explicit Intercept(Navigator *navigator);

	void on_inactive() override;
	void on_activation() override;
	void on_active() override;

private:
	enum class Phase : uint8_t {
		Idle = 0,
		Entry,
		RunIn,
		Egress,
		Complete
	};

	bool update_plan(bool force);
	bool plan_valid() const;

	void publish_entry_setpoint();
	void publish_runin_setpoint();
	void publish_egress_setpoint();

	void publish_triplet_from_item(const mission_item_s &item,
				       double prev_lat, double prev_lon, float prev_alt_amsl);

	void compute_entry_egress();

	uORB::Subscription _intercept_setpoint_sub{ORB_ID(intercept_setpoint)};
	intercept_setpoint_s _intercept_sp{};

	Phase _phase{Phase::Idle};
	hrt_abstime _phase_started{0};

	// Cached geometry
	double _entry_lat{NAN};
	double _entry_lon{NAN};
	float  _entry_alt_amsl{NAN};

	double _intercept_lat{NAN};
	double _intercept_lon{NAN};
	float  _intercept_alt_amsl{NAN};

	double _egress_lat{NAN};
	double _egress_lon{NAN};
	float  _egress_alt_amsl{NAN};
};
