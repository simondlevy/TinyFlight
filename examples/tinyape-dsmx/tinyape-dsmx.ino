/*
   Based on https://github.com/nickrehm/dRehmFlight
 */

#include <Wire.h> 

#include <dsmrx.hpp>

#include <I2Cdev.h>
#include <MPU6050.h>

#include <oneshot125.hpp>
#include <vector>

#include <datatypes.h>
#include <mixers.hpp>
#include <utils.hpp>


#include <tasks/blink.hpp>
#include <tasks/debug.hpp>

#include <madgwick.hpp>

#include <tasks/ekf_predict.hpp>
#include <ekf.hpp>

#include <closed_loop/pitch_roll_angle.hpp>
#include <closed_loop/pitch_roll_rate.hpp>
#include <closed_loop/yaw_angle.hpp>
#include <closed_loop/yaw_rate.hpp>

static const bool USE_EKF = false;

static const auto DEBUG_TASK = DebugTask::RADIO;

// PID Control constants -----------------------------------------------------

static const float PITCH_ROLL_ANGLE_KP = 0.2;    

static const float YAW_RATE_KP = 1.20e-2;

static const float ANGLE_MAX = 240;
static const float LPF_B = 0.9;
static const float LPF_L = 30;

static const float PITCH_ROLL_RATE_KP = 0.0015;    
static const float PITCH_ROLL_RATE_KD = 0.000002;  

// ---------------------------------------------------------------------------

static const uint32_t EKF_PREDICT_RATE = 500;

// MPU6050 gyro and accel full scale value selection and scale factor
static const uint8_t GYRO_SCALE = MPU6050_GYRO_FS_2000;
static const uint8_t ACCEL_SCALE = MPU6050_ACCEL_FS_8;

// IMU biases obtained from calibration;
static const axis3_t GYRO_BIAS = {0, 0, 0};
static const axis3_t ACCEL_BIAS = {0, 0, 0};

// Do not exceed 2000Hz, all filter paras tuned to 2000Hz by default
static const uint32_t INNER_LOOP_FREQ = 2000;

// We kill the motors if we haven't had a radio signal in this many
// microseconds
static const uint32_t FAILSAFE_TIMEOUT_USEC = 50000;

// IMU LPF params
static const float B_GYRO = 0.1;    
static const float B_ACCEL = 0.14;   


// Das Blinkenlights
static const float    BLINK_RATE_HZ = 1.5;
static BlinkTask _blinkTask;

// Throttle stick tolerance
static const uint32_t THROTTLE_DOWN = 1050;

// Debugging
static const float DEBUG_RATE_HZ = 100;
static DebugTask _debugTask;

// We can use an EKF or Madgwick filter for state estimation -----------------

static Ekf _ekf;
static EkfPredictTask _ekfPredictTask;
//static EkfUpdateTask _ekfUpdateTask;

static MadgwickFilter _madgwick;

// We use Crazyflie QuadX motor layout ----------------------------------------

static const std::vector<uint8_t> MOTOR_PINS = { 1, 2, 3, 0 };

static auto _motors = OneShot125(MOTOR_PINS);

// Motor pulse widths for OneShot125
static uint8_t _m1_usec, _m2_usec, _m3_usec, _m4_usec; 

// MPU6050 IMU ----------------------------------------------------------------

static MPU6050 _mpu6050;

// Spektrum DSMX receiver ----------------------------------------------------

static const uint8_t RX_CHANNELS = 8;

Dsm2048 _rx;

void serialEvent2(void)
{
    while (Serial2.available()) {
        _rx.parse(Serial2.read(), micros());
    }
}

// Closed-loop (PID) controllers ---------------------------------------------

PitchRollAngleController _pitchRollAngleController;
PitchRollRateController _pitchRollRateController;
YawAngleController _yawAngleController;
YawRateController _yawRateController;

// Local helpers =============================================================

static void armMotor(uint8_t & m_usec)
{
    // OneShot125 range from 125 to 250 usec
    m_usec = 125;
}

static uint8_t scaleMotor(const float mval)
{
    return Utils::u8constrain(mval*125 + 125, 125, 250);

}

static void initImu() 
{
    Wire.begin();
    Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...

    _mpu6050.initialize();

    if (_mpu6050.testConnection() == false) {
        Serial.printf("MPU6050 initialization unsuccessful:\n");
        Serial.printf("Check MPU6050 wiring or try cycling power\n");
        while(true) {}
    }

    //From the reset state all registers should be 0x00, so we should be at
    //max sample rate with digital low pass filter(s) off.  All we need to
    //do is set the desired fullscale ranges
    _mpu6050.setFullScaleGyroRange(GYRO_SCALE);
    _mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
}

static void smootheImuAxis(
        const int16_t raw,
        const float scale_factor,
        const float bias,
        const float b,
        float & curr,
        float & prev)
{
    // Scale; subtract bias; run low-pass filter
    curr = (1- b) * prev + b * (raw / scale_factor - bias);

    prev = curr;
}

static void smootheImuSensor(
        const int16_t xraw, 
        const int16_t yraw, 
        const int16_t zraw, 
        const float scale_factor,
        const float b,
        const axis3_t bias,
        axis3_t & sensor,
        axis3_t & sensor_prev)
{
    smootheImuAxis(xraw, scale_factor, bias.x, b, sensor.x, sensor_prev.x);
    smootheImuAxis(yraw, scale_factor, bias.y, b, sensor.y, sensor_prev.y);
    smootheImuAxis(zraw, scale_factor, bias.z, b, sensor.z, sensor_prev.z);
}

static void readImu(axis3_t & gyro, axis3_t & accel)
{
    int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;

    _mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    static axis3_t _gyro_prev;

    // Convert raw gyro to smoothed deg/sec
    smootheImuSensor(gx, gy, gz, 
            131.f / (1 << GYRO_SCALE),
            B_GYRO, GYRO_BIAS, gyro, _gyro_prev);

    static axis3_t _accel_prev;

    // Convert raw accel to smoothed gs
    smootheImuSensor(ax, ay, az, 
            ACCEL_SCALE * 2048.f,
            B_ACCEL, ACCEL_BIAS, accel, _accel_prev);
}

static float channelToDemand(
        const float chan, const float offset=1500, const float denom=500)
{
    return (chan - offset) / denom;
}

static void getOpenLoopDemands(const channels_t & channels, demands_t & demands)
{
    demands.thrust = channelToDemand(channels.c1, 1000, 1000);
    demands.roll = channelToDemand(channels.c2);
    demands.pitch = channelToDemand(channels.c3);
    demands.yaw = channelToDemand(channels.c4);

    // Constrain roll, pitch demand angles to 30 degrees
    demands.thrust = constrain(demands.thrust, 0.0, 1.0);
    demands.roll = constrain(demands.roll, -1.0, 1.0) * 30;
    demands.pitch = constrain(demands.pitch, -1.0, 1.0) * 30;
    demands.yaw = constrain(demands.yaw, -1.0, 1.0);
}

static void scaleMotors(const quad_motors_t & motors)
{
    _m1_usec = scaleMotor(motors.m1);
    _m2_usec = scaleMotor(motors.m2);
    _m3_usec = scaleMotor(motors.m3);
    _m4_usec = scaleMotor(motors.m4);
}

static void readReceiver(
        const uint32_t usec_curr, 
        channels_t & channels,
        bool & gotFailsafe) 
{
    if (_rx.timedOut(micros())) {
        gotFailsafe = true;
    }

    else if (_rx.gotNewFrame()) {

        uint16_t values[RX_CHANNELS];

        _rx.getChannelValues(values, RX_CHANNELS);

        channels.c1 = values[0];
        channels.c2 = values[1];
        channels.c3 = values[2];
        channels.c4 = values[3];
        channels.c5 = values[4];
        channels.c6 = values[5];
    }
}

static void runMotors(void) 
{
    _motors.set(0, _m1_usec);
    _motors.set(1, _m2_usec);
    _motors.set(2, _m3_usec);
    _motors.set(3, _m4_usec);

    _motors.run();
}

static void cutMotors(
        const uint32_t c5, const bool gotFailsafe, bool & isArmed) 
{
    if (c5 < 1500 || !isArmed || gotFailsafe) {
        isArmed = false;
        _m1_usec = 120;
        _m2_usec = 120;
        _m3_usec = 120;
        _m4_usec = 120;
    }
}

static void runLoopDelay(const uint32_t usec_curr) 
{
    //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
    /*
     * It's good to operate at a constant loop rate for filters to remain
     * stable and whatnot. Interrupt routines running in the background cause
     * the loop rate to fluctuate. This function basically just waits at the
     * end of every loop iteration until the correct time has passed since the
     * start of the current loop for the desired loop rate in Hz. 2kHz is a
     * good rate to be at because the loop nominally will run between 2.8kHz -
     * 4.2kHz. This lets us have a little room to add extra computations and
     * remain above 2kHz, without needing to retune all of our filtering
     * parameters.
     */
    float invFreq = 1.0 / INNER_LOOP_FREQ * 1000000.0;
    uint32_t checker = micros();

    //Sit in loop until appropriate time has passed
    while (invFreq > (checker - usec_curr)) {
        checker = micros();
    }
}

static void blinkOnStartup(void)
{
    // These constants are arbitrary, so we hide them here
    static const uint8_t  BLINK_INIT_COUNT = 10;
    static const uint32_t BLINK_INIT_TIME_MSEC = 100;

    for (uint8_t j=0; j<BLINK_INIT_COUNT; j++) {
        digitalWrite(LED_BUILTIN, LOW);
        delay(BLINK_INIT_TIME_MSEC);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(BLINK_INIT_TIME_MSEC);
    }
}

static void getVehicleState(
        const uint32_t usec,
        const float dt,
        const axis3_t & gyro,
        const axis3_t & accel,
        state_t & state)
{
    static quat_t _quat;

    if (USE_EKF) {

        _ekfPredictTask.run(usec, EKF_PREDICT_RATE, _ekf);

        _ekf.accumulate_gyro(gyro);
        _ekf.accumulate_accel(accel);

        //_ekf.update_with_range();
        //_ekf.update_with_flow();
        //_ekf.finalize();

        axis3_t pos = {};
        axis3_t dpos = {};

        _ekf.get_vehicle_state(_quat, pos, dpos);
    }

    else {

        _madgwick.getQuat(dt, gyro, accel, _quat);
    }

    Utils::quat2euler(_quat, state.ang);

    // Get angular velocities directly from gyro.  We swap the X and Y axes and 
    // negate Y and Z for nose-down, nose-right positive.
    state.dang.x = gyro.y;  
    state.dang.y = -gyro.x; 
    state.dang.z = -gyro.z; 
}

// Main program ===============================================================

void setup() 
{
    // Initialize USB port for debugging
    Serial.begin(500000);
    delay(500);

    // Initialize UART for receiver
    Serial2.begin(115000);

    // Initialize LED
    pinMode(LED_BUILTIN, OUTPUT); 

    // Initialize IMU communication
    initImu();

    // Initialize state estimator
    _ekf.initialize();
    _madgwick.init();

    // Arm OneShot125 motors
    armMotor(_m1_usec);
    armMotor(_m2_usec);
    armMotor(_m3_usec);
    armMotor(_m4_usec);
    _motors.arm();

    //Indicate entering main loop with some quick blinks
    blinkOnStartup(); 
}

static void damp(float & curr, const float prev)
{            
    curr = LPF_L * curr;
    curr = Utils::fconstrain(curr, -ANGLE_MAX, ANGLE_MAX);
    curr = (1.0 - LPF_B) * prev + LPF_B * curr;
}

static void runClosedLoop(
        const state_t & state,
        const float dt,
        const bool reset,
        demands_t & demands)
{
    static float _roll_des_prev;

    static float _pitch_des_prev;

    _pitchRollAngleController.run( PITCH_ROLL_ANGLE_KP, state, dt, demands);

    damp(demands.roll, _roll_des_prev);
    damp(demands.pitch, _pitch_des_prev);

    _pitchRollRateController.run(
            PITCH_ROLL_RATE_KP, PITCH_ROLL_RATE_KD, state, dt, reset, demands);

    _yawAngleController.run(state, dt, demands);

    _yawRateController.run(YAW_RATE_KP, state, dt, demands);
}


void loop() 
{
    // Receivers channels are sampled as available and so must persist between
    // loop calls
    static channels_t _channels;

    // Safety
    static bool _isArmed;
    static bool _gotFailsafe;

    // Keep track of what time it is and how much time has elapsed since the last loop
    static uint32_t usec_prev;
    const auto usec_curr = micros();      
    const auto dt = (usec_curr - usec_prev)/1e6;
    usec_prev = usec_curr;      

    // LED should be on when armed
    if (_isArmed) {
        digitalWrite(LED_BUILTIN, HIGH);
    }

    // Otherwise, blink LED as heartbeat
    else {
        _blinkTask.run(usec_curr, BLINK_RATE_HZ);
    }

    // Read channel values from receiver
    readReceiver(usec_curr, _channels, _gotFailsafe);

    // Check if the thrust cut is off and the thrust input is low to
    // prepare for flight.
    if ((_channels.c5 > 1500) && (_channels.c1 < THROTTLE_DOWN)) {
        _isArmed = true;
    }

    // Read IMU
    axis3_t gyro = {};
    axis3_t accel = {};
    readImu(gyro, accel);

    state_t state = {};

    // Get vehiclestate from estimator
    getVehicleState(usec_curr, dt, gyro, accel, state);

    // Convert channel value to open-loop demands
    demands_t demands;
    getOpenLoopDemands(_channels, demands);

    // Reset PID controllers when throttle is down
    const auto throttleIsDown = _channels.c1 < THROTTLE_DOWN;

    // Run PID controllers to add closed-loop control to open-loop demands
    runClosedLoop(state, dt, throttleIsDown, demands);

    // Run mixer to get motor values from closed-loop demands
    quad_motors_t motors = {};
    Mixer::runCF(demands, motors);

    // Rescale motor values for OneShot125
    scaleMotors(motors);

    // Turn off motors under various conditions
    cutMotors(_channels.c5, _gotFailsafe, _isArmed); 

    // Run motors
    runMotors();

    // Regulate loop rate
    runLoopDelay(usec_curr); 

    // Debug by periodically printing important values
    _debugTask.run(usec_curr, DEBUG_RATE_HZ, DEBUG_TASK,
            _channels, demands, state, gyro, accel, motors);

}
