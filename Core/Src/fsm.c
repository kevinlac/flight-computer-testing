#include "fsm.h"
#include <math.h>
#include <string.h>

static void fsm_transition(FSM_t *fsm, FSM_State_t newState,
                           uint32_t currentTime_ms) {
    fsm->state = newState;

    switch (newState) {
    case FSM_STATE_READY:
        break;

    case FSM_STATE_THRUST:
        fsm->launchTime_ms = currentTime_ms;
        break;

    case FSM_STATE_BURNOUT:
        fsm->burnoutTime_ms = currentTime_ms;
        break;

    case FSM_STATE_APOGEE:
    case FSM_STATE_SECOND_CHUTE:
    case FSM_STATE_LANDED:
    default:
        break;
    }

    if (fsm->actionCb != NULL) {
        fsm->actionCb(newState, currentTime_ms, fsm->actionCtx);
    }
}

/* Generic helper: given whether a raw threshold condition holds RIGHT NOW,
 * update the tracking state for it, and return true once it has
 * held continuously for at least minDurationMs. Any break in the
 * condition resets the timer - this is what makes a single spike (or a
 * short noise burst) insufficient regardless of sample rate. */
static bool fsm_update_duration_condition(bool conditionNow, bool *active,
                                          uint32_t *startMs,
                                          uint32_t minDurationMs,
                                          uint32_t currentTime_ms) {
    if (conditionNow) {
        if (!*active) {
            *active = true;
            *startMs = currentTime_ms;
        }
        return (currentTime_ms - *startMs) >= minDurationMs;
    } else {
        *active = false;
        return false;
    }
}

static bool fsm_check_thrust(FSM_t *fsm, float IMUaccel_y,
                             uint32_t currentTime_ms) {
    // based on blue raven data, it goes to above 10g sustained,
    // our thing will saturate at like 4g, so we'll put it at 3g
    return fsm_update_duration_condition(
        IMUaccel_y > 3000.0f, &fsm->thrustConditionActive,
        &fsm->thrustConditionStartMs, fsm->minThrustDurationMs, currentTime_ms);
}

static bool fsm_check_burnout(FSM_t *fsm, float IMUaccel_y,
                              uint32_t currentTime_ms) {
    // based on blue raven data, after the initial 10g thrust period, it drops
    // to like ~-2, then goes to like 0
    return fsm_update_duration_condition(
        IMUaccel_y < -1000.0f, &fsm->burnoutConditionActive,
        &fsm->burnoutConditionStartMs, fsm->minBurnoutDurationMs,
        currentTime_ms);
}

static bool fsm_check_apogee(FSM_t *fsm, float IMUaccel_y,
                             uint32_t currentTime_ms) {
    return fsm_update_duration_condition(
        IMUaccel_y < 0.0f, &fsm->apogeeConditionActive,
        &fsm->apogeeConditionStartMs, fsm->minApogeeDurationMs, currentTime_ms);
}

static bool fsm_check_landed(FSM_t *fsm, float IMUgyro_x, float IMUgyro_y,
                             float IMUgyro_z, uint32_t currentTime_ms) {
    bool stable = fabsf(IMUgyro_x) < fsm->tolerance &&
                  fabsf(IMUgyro_y) < fsm->tolerance &&
                  fabsf(IMUgyro_z) < fsm->tolerance;

    /* NOTE: original pseudocode never updated these, so "landed" could only
     * ever be judged against a stale (0,0,0) baseline. Update every call. */
    fsm->prevGyroX = IMUgyro_x;
    fsm->prevGyroY = IMUgyro_y;
    fsm->prevGyroZ = IMUgyro_z;

    return fsm_update_duration_condition(
        stable, &fsm->landedConditionActive, &fsm->landedConditionStartMs,
        fsm->minLandedDurationMs, currentTime_ms);
}

void FSM_Init(FSM_t *fsm) {
    memset(fsm, 0, sizeof(*fsm));
    fsm->state = FSM_STATE_READY;
    /* secondChuteAltitude is in FEET (see the unit note on FSM_t above).
     * 900.0f matches this project's actual target main-deploy altitude -
     * still update this if that target changes. */
    fsm->secondChuteAltitude = 900.0f;

    /* Placeholder fallback timeouts, in case sensor-based detection ever
     * fails to fire. These are LAST-RESORT safety nets, not meant to be
     * the normal way transitions happen - they need to be comfortably
     * longer than your motor's real burn time / real coast-to-apogee
     * time (with margin), or they'll preempt real sensor detection
     * exactly like the original 1234ms values did. Get real burn time
     * from your motor's datasheet and real coast time from an OpenRocket
     * (or similar) sim of this specific rocket/motor/mass combo, then
     * set these to something like 1.5-2x that as headroom. */
    fsm->maxExpectedThrustMs = 8000;
    fsm->maxExpectedCoastMs = 45000;
    fsm->tolerance = 10000.0f; // 10 degrees per second gyro
    fsm->haveStartingAltitude = false;

    /* Placeholder durations - same "needs real tuning" caveat as every
     * other constant in here. These are just plausible starting points:
     * ~100ms of sustained accel is enough to reject a single bump/spike
     * but still react fast to genuine motor ignition/burnout/apogee;
     * "landed" intentionally requires a much longer, multi-second still
     * period since a rocket settling under a canopy can wobble briefly
     * right after touchdown. */
    fsm->minThrustDurationMs = 100;
    fsm->minBurnoutDurationMs = 100;
    fsm->minApogeeDurationMs = 100;
    fsm->minLandedDurationMs = 3000;
}

void FSM_SetStartingAltitude(FSM_t *fsm, float startingAltitude_ft) {
    fsm->startingAltitude = startingAltitude_ft;
    fsm->haveStartingAltitude = true;
}

void FSM_SetActionCallback(FSM_t *fsm, FSM_ActionCallback_t cb, void *ctx) {
    fsm->actionCb = cb;
    fsm->actionCtx = ctx;
}

bool FSM_NewReading(FSM_t *fsm, int8_t accel_x, int8_t accel_y, int8_t accel_z,
                    float IMUaccel_x, float IMUaccel_y, float IMUaccel_z,
                    float IMUgyro_x, float IMUgyro_y, float IMUgyro_z,
                    float altitude_ft, uint32_t currentTime_ms) {
    (void)accel_x;
    (void)accel_y;
    (void)accel_z;
    (void)IMUgyro_x;
    (void)IMUgyro_y;
    (void)IMUgyro_z;

    if (!fsm->haveStartingAltitude) {
        FSM_SetStartingAltitude(fsm, altitude_ft);
    }

    if (altitude_ft > fsm->maxReachedAltitude) {
        fsm->maxReachedAltitude = altitude_ft;
    }

    FSM_State_t before = fsm->state;

    switch (fsm->state) {
    case FSM_STATE_READY: {
        bool thrustSustained =
            fsm_check_thrust(fsm, IMUaccel_y, currentTime_ms);
        if (thrustSustained && (altitude_ft - fsm->startingAltitude > 10.0f)) {
            fsm_transition(fsm, FSM_STATE_THRUST, currentTime_ms);
        }
        break;
    }

    case FSM_STATE_THRUST: {
        bool burnoutSustained =
            fsm_check_burnout(fsm, IMUaccel_y, currentTime_ms);
        if (burnoutSustained ||
            (currentTime_ms - fsm->launchTime_ms) > fsm->maxExpectedThrustMs) {
            fsm_transition(fsm, FSM_STATE_BURNOUT, currentTime_ms);
        }
        break;
    }

    case FSM_STATE_BURNOUT: {
        bool velocityCond = fsm_check_apogee(fsm, IMUaccel_y, currentTime_ms);

        bool tiltCond = false; /* TODO: flag if tilt has changed > 90 deg (needs
                                  gyro integration) */
        /* NOTE: this 10.0f margin is now 10 FEET (it used to be 10 meters
         * back when altitude was in meters, i.e. it just got ~3x tighter).
         * Worth widening if baro noise causes false triggers here. */
        bool heightCond = (altitude_ft + 10.0f) < fsm->maxReachedAltitude;
        int votes =
            (tiltCond ? 1 : 0) + (heightCond ? 1 : 0) + (velocityCond ? 1 : 0);

        if (votes >= 2 ||
            (currentTime_ms - fsm->burnoutTime_ms) > fsm->maxExpectedCoastMs) {
            fsm_transition(fsm, FSM_STATE_APOGEE, currentTime_ms);
        }
        break;
    }

    case FSM_STATE_APOGEE:
        if ((altitude_ft + 10.0f) < fsm->secondChuteAltitude) {
            fsm_transition(fsm, FSM_STATE_SECOND_CHUTE, currentTime_ms);
        }
        break;

    case FSM_STATE_SECOND_CHUTE: {
        bool landedSustained = fsm_check_landed(fsm, IMUgyro_x, IMUgyro_y,
                                                IMUgyro_z, currentTime_ms);
        if (landedSustained) {
            fsm_transition(fsm, FSM_STATE_LANDED, currentTime_ms);
        }
        break;
    }

    case FSM_STATE_LANDED:
    default:
        break;
    }

    return fsm->state != before;
}

FSM_State_t FSM_GetState(const FSM_t *fsm) { return fsm->state; }

const char *FSM_StateName(FSM_State_t state) {
    switch (state) {
    case FSM_STATE_READY:
        return "READY";
    case FSM_STATE_THRUST:
        return "THRUST";
    case FSM_STATE_BURNOUT:
        return "BURNOUT";
    case FSM_STATE_APOGEE:
        return "APOGEE";
    case FSM_STATE_SECOND_CHUTE:
        return "SECOND_CHUTE";
    case FSM_STATE_LANDED:
        return "LANDED";
    default:
        return "UNKNOWN";
    }
}
