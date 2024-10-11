#include <Arduino.h>
#include "estimate_kalman.h"
#include "Var_types.h"

#define PREDICT_RATE RATE_100_HZ // Slower that the IMU rate -- 1000 Hz
const uint32_t PREDICTION_UPDATE_INTERVAL = 1000/PREDICT_RATE; // in ms

#define MAX_COMVARIANCE (100)
#define MIN_COMVARIANCE (1e-6f)


// Kalmain Filter sensors to use:
#define KALMAN_USE_BARO false
#define KALMAN_USE_GPS false
#define KALMAN_USE_FLOW false
#define KALMAN_USE_LIDAR false






static vec3_t gyroLatest;
static vec3_t accLatest;

// indicate that the estimated state is corrupted and the kalman filter should be reset
bool resetEstimation = false;

static KalmanCoreParams_t coreParams;
// Estimated state. sent to the main loop 
static state_t stateEstimation;

#define ONE_SECOND 1000

static rateSupervisor_t rateSupervisorContext;

#define WARNING_BLOCK_TIME_MS 2000
#define uint32_t warningBlockTimeMS = 0;

#ifdef KALMAN_USE_BARO
static const bool useBaro = true;
#else
static const bool useBaro = false;
#endif