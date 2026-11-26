# Per-sector speed caps from cp_brake to the limiter hook (collision-prevention ICD-CP-04)
uint64 timestamp		# [us] publication time, time since system start
uint64 sample_timestamp		# [us] acquisition time of the map the caps come from, PX4 time

float32 heading_rad		# [rad] vehicle heading at sample_timestamp, rotates the body FRD caps to local NED
bool heading_valid		# false when no heading exists for sample_timestamp

float32[72] caps_m_s		# [m/s] speed cap per sector, body FRD, sector 0 forward, clockwise, >= 0
float32[72] distances_m		# [m] distance whose stopping speed is the cap, per sector (ADR 0025)
float32 clearance_m		# [m] CPB_MARGIN, the half-width of the corridor of ADR 0025
bool enforcement_active		# the hook may act on the caps
bool valid			# the caps may be used; when false the hook follows the ICD-CP-04 fallbacks

uint8 state
uint8 STATE_DISABLED = 0
uint8 STATE_OBSERVER_ONLY = 1
uint8 STATE_NOT_READY = 2
uint8 STATE_ACTIVE_CLEAR = 3
uint8 STATE_BRAKING = 4
uint8 STATE_SAFE_STOP = 5

uint32 reason_flags		# why the state is what it is (the lib/cp_brake ReasonFlags)
uint32 REASON_INVALID_CONFIG = 1
uint32 REASON_FRAME_INVALID = 2
uint32 REASON_DATA_INVALID = 4
uint32 REASON_SAMPLE_STALE = 8		# no accepted map, or its age reached CPB_AGE_STOP
uint32 REASON_NO_HEADING = 16		# no heading for sample_timestamp
uint32 REASON_UNQUALIFIED = 32		# the PX4 estimator or heading is invalid, no CP action (ADR 0007)
uint32 REASON_BLOCKED = 64		# the caps leave no horizontal motion
uint32 REASON_ACK_REQUIRED = 128	# a safe stop is latched and waits for the pilot acknowledgement
uint32 REASON_OVERRIDE = 256		# a pilot override is active, the constraint is not enforced
uint32 REASON_TIME_SYNC = 512		# the time alignment of the latest map is not good, the map is not used
uint32 REASON_CONSTRAINT_STALE = 1024	# set by the hook: no constraint, one older than CPB_CSTR_AGE, or no usable age limit
uint32 REASON_CONSTRAINT_MALFORMED = 2048	# set by the hook: a heading that is not a number, or an enforced state it cannot map
uint32 REASON_NAV_STATE = 4096		# set by the hook: transparent, the navigation state is neither Position nor Mission
uint32 REASON_UNCLAIMED_DIRECTION = 8192	# set by the hook: the motion points beyond CPB_REAR_ANG, outside the claim (limits nothing)
