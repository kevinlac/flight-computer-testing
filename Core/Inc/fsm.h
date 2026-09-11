#ifndef FSM_H
#define FSM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FSM_STATE_READY = 0,
    FSM_STATE_THRUST,
    FSM_STATE_BURNOUT,
    FSM_STATE_APOGEE,
    FSM_STATE_SECOND_CHUTE,
    FSM_STATE_LANDED
} FSM_State_t;

/* Called exactly once, every time the FSM transitions into a new state.
 * For live flight, wire this to real GPIO toggles (deploy drogue, deploy
 * main, etc). For bench/replay testing of old logs, wire it to a
 * printf-only stub so nothing physical ever fires. See main.c example. */
typedef void (*FSM_ActionCallback_t)(FSM_State_t newState,
                                     uint32_t currentTime_ms, void *ctx);

/* ALL altitude values in this struct and in FSM_NewReading's altitude_ft
 * parameter are in FEET (not meters) - matches the sensor's native units,
 * so there's no unit conversion step anywhere to accidentally get wrong. */
typedef struct {
    FSM_State_t state;

    /* Each of these four conditions is now tracked as "how long has this
     * been continuously true", not "how many samples in a row" - so
     * detection is agnostic to how fast the sensor is actually sampling.
     * *ConditionActive is true while the raw threshold check currently
     * holds; *ConditionStartMs is the timestamp it most recently started
     * holding (reset to the current time any time the condition breaks). */
    bool thrustConditionActive;
    uint32_t thrustConditionStartMs;
    uint32_t minThrustDurationMs;

    bool burnoutConditionActive;
    uint32_t burnoutConditionStartMs;
    uint32_t minBurnoutDurationMs;

    bool apogeeConditionActive;
    uint32_t apogeeConditionStartMs;
    uint32_t minApogeeDurationMs;

    bool landedConditionActive;
    uint32_t landedConditionStartMs;
    uint32_t minLandedDurationMs;

    bool haveStartingAltitude;
    float startingAltitude;
    float maxReachedAltitude;
    float secondChuteAltitude;

    uint32_t launchTime_ms;
    uint32_t burnoutTime_ms;
    uint32_t maxExpectedThrustMs;
    uint32_t maxExpectedCoastMs;

    float prevGyroX, prevGyroY, prevGyroZ;
    float tolerance;

    FSM_ActionCallback_t actionCb;
    void *actionCtx;
} FSM_t;

/* Zero-initializes fsm and sets default thresholds/state = READY. */
void FSM_Init(FSM_t *fsm);

/* Optional: force the pad/ground altitude (in feet) instead of letting the
 * first FSM_NewReading() call capture it automatically. Call this AFTER
 * FSM_Init. */
void FSM_SetStartingAltitude(FSM_t *fsm, float startingAltitude_ft);

/* Optional: register a callback fired on every state transition. */
void FSM_SetActionCallback(FSM_t *fsm, FSM_ActionCallback_t cb, void *ctx);

/* Feed one sensor sample into the state machine.
 * currentTime_ms must be monotonically increasing (HAL_GetTick() live,
 * or telemetry.timestamp_ms when replaying a logged flight).
 * Returns true if this call caused a state transition. */
bool FSM_NewReading(FSM_t *fsm, int8_t accel_x, int8_t accel_y, int8_t accel_z,
                    float IMUaccel_x, float IMUaccel_y, float IMUaccel_z,
                    float IMUgyro_x, float IMUgyro_y, float IMUgyro_z,
                    float altitude_ft, uint32_t currentTime_ms);

FSM_State_t FSM_GetState(const FSM_t *fsm);
const char *FSM_StateName(FSM_State_t state);

#ifdef __cplusplus
}
#endif

#endif /* FSM_H */
