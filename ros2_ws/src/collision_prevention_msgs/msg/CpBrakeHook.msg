# What the limiter hook did when it applied a cp_brake_constraints message (collision-prevention ICD-CP-04, ICD-CP-05)
# Published by mc_pos_control, apart from cp_brake_status: a topic with two publishers is no longer advertised once
# one of them unadvertises it, which would hide this message exactly when the module stopped.
uint64 timestamp		# [us] the time the hook applied the constraint, so sample_age_us is sample to enforcement
uint64 sample_timestamp		# [us] acquisition time of the map the constraint comes from, PX4 time, 0 when no constraint was received
uint64 constraint_timestamp	# [us] publication time of the constraint the hook used, 0 when none was received

uint32 sample_age_us		# [us] timestamp minus sample_timestamp
uint32 constraint_age_us	# [us] timestamp minus constraint_timestamp

uint8 state			# state of the constraint, see CpBrakeConstraints STATE_*
uint32 reason_flags		# see CpBrakeConstraints REASON_*: those of the constraint when it was used, plus the hook's own

uint8 action			# what the hook did, ACTION_*
uint8 ACTION_TRANSPARENT = 0	# CPB_EN off, before flight, or a navigation state other than Position or Mission (reason NAV_STATE)
uint8 ACTION_NO_ACTION = 1	# the constraint says no action (observer-only, or an unqualified PX4 state)
uint8 ACTION_CLEAR = 2		# the limiter ran and no setpoint exceeded a cap
uint8 ACTION_LIMITED = 3	# the limiter reduced the velocity setpoint and cleared the horizontal feed-forward
uint8 ACTION_SAFE_STOP = 4	# zero horizontal velocity and feed-forward cleared

uint8 active_sector		# sector whose cap cut the velocity setpoint most, 255 for none (with ACTION_LIMITED: the CPB_V_SIDE speed lock)
float32[2] vel_sp_in		# [m/s] horizontal velocity setpoint entering the hook, local NED (north, east)
float32[2] vel_sp_out		# [m/s] horizontal velocity setpoint leaving the hook
float32[2] acc_sp_in		# [m/s^2] horizontal feed-forward acceleration entering the hook, NaN when unset
float32[2] acc_sp_out		# [m/s^2] horizontal feed-forward acceleration leaving the hook
float32[2] vel			# [m/s] horizontal velocity of the vehicle as the hook saw it, local NED
