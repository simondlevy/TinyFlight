#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <time.h>

#include <datatypes.h>
#include <utils.hpp>

#include <tasks/core.hpp>

#include "sticks.hpp"

// ----------------------------------------------------------------------------

static const float PITCH_ROLL_ANGLE_KP = 6e0;

static const float PITCH_ROLL_RATE_KP = 1.25e-2;
static const float PITCH_ROLL_RATE_KD = 1.25e-4;

static const float YAW_RATE_KP = 1.20e-2;

static const float THROTTLE_TOLERANCE = 0.1;

// Motor thrust constants for climb-rate PID controller
static const float TBASE = 56;
static const float TSCALE = 0.25;
static const float TMIN = 0;

// https://www.bitcraze.io/documentation/tutorials/getting-started-with-flow-deck/
static const float ALTITUDE_TARGET_INITIAL = 0.4;
static const float ALTITUDE_TARGET_MIN = 0.2;
static const float ALTITUDE_TARGET_MAX = 2.0;  // 3.0 in original

// Arbitrary time constant
static const float DT = .01;

// Motors
static WbDeviceTag _m1_motor;
static WbDeviceTag _m2_motor;
static WbDeviceTag _m3_motor;
static WbDeviceTag _m4_motor;

// ---------------------------------------------------------------------------

static Sticks _sticks;

static WbDeviceTag _makeMotor(const char * name, const float direction)
{
    auto motor = wb_robot_get_device(name);

    wb_motor_set_position(motor, INFINITY);
    wb_motor_set_velocity(motor, direction);

    return motor;
}

static void getVehicleState(
        const WbDeviceTag & gyro, 
        const WbDeviceTag & imu, 
        const WbDeviceTag & gps,
        state_t & state)
{
    // Track previous time and position for calculating motion
    static float tprev;
    static float xprev;
    static float yprev;
    static float zprev;

    const auto tcurr = wb_robot_get_time();
    const auto dt =  tcurr - tprev;
    tprev = tcurr;

    // Get yaw angle in radians
    auto psi = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];

    // Get state variables, negating gyro for nose-right positive
    state.pos.z  = wb_gps_get_values(gps)[2];
    state.ang.x  =  Utils::RAD2DEG*(wb_inertial_unit_get_roll_pitch_yaw(imu)[0]);
    state.dang.x =  Utils::RAD2DEG*(wb_gyro_get_values(gyro)[0]);
    state.ang.y  =  Utils::RAD2DEG*(wb_inertial_unit_get_roll_pitch_yaw(imu)[1]);
    state.dang.y =  Utils::RAD2DEG*(wb_gyro_get_values(gyro)[1]); 
    state.ang.z  =  -Utils::RAD2DEG*(psi); 
    state.dang.z =  -Utils::RAD2DEG*(wb_gyro_get_values(gyro)[2]);

    // Use temporal first difference to get world-cooredinate velocities
    auto x = wb_gps_get_values(gps)[0];
    auto y = wb_gps_get_values(gps)[1];
    auto dx = (x - xprev) / dt;
    auto dy = (y - yprev) / dt;
    state.dpos.z = (state.pos.z - zprev) / dt;

    // Rotate X,Y world velocities into body frame to simulate optical-flow
    // sensor
    auto cospsi = cos(psi);
    auto sinpsi = sin(psi);
    state.dpos.x = dx * cospsi + dy * sinpsi;
    state.dpos.y = dx * sinpsi - dy * cospsi;

    // Save past time and position for next time step
    xprev = x;
    yprev = y;
    zprev = state.pos.z;
}

static WbDeviceTag _makeSensor(
        const char * name, 
        const uint32_t timestep,
        void (*f)(WbDeviceTag tag, int sampling_period))
{
    auto sensor = wb_robot_get_device(name);
    f(sensor, timestep);
    return sensor;
}

int main(int argc, char ** argv)
{
    wb_robot_init();

    const int timestep = (int)wb_robot_get_basic_time_step();

    // Initialize motors
    _m1_motor = _makeMotor("m1_motor", +1);
    _m2_motor = _makeMotor("m2_motor", -1);
    _m3_motor = _makeMotor("m3_motor", +1);
    _m4_motor = _makeMotor("m4_motor", -1);

    // Initialize sensors
    auto imu = _makeSensor("inertial_unit", timestep, wb_inertial_unit_enable);
    auto gyro = _makeSensor("gyro", timestep, wb_gyro_enable);
    auto gps = _makeSensor("gps", timestep, wb_gps_enable);
    auto camera = _makeSensor("camera", timestep, wb_camera_enable);

    _sticks.init();

    float altitudeTarget = ALTITUDE_TARGET_INITIAL;

    demands_t demands = {};

    CoreTask coreTask = {};

    coreTask.init(
            PITCH_ROLL_ANGLE_KP, 
            PITCH_ROLL_RATE_KP, 
            PITCH_ROLL_RATE_KD,
            YAW_RATE_KP, 
            TBASE, 
            TSCALE, 
            TMIN,
            DT);

    while (wb_robot_step(timestep) != -1) {

        //Un-comment if you want to try OpenCV
        // runCamera(camera);

        // Get open-loop demands from input device (keyboard, joystick, etc.)
        _sticks.read(demands.thrust, demands.roll, demands.pitch, demands.yaw);

        // Get vehicle state from sensors
        state_t state = {};
        getVehicleState(gyro, imu, gps, state);

        quad_motors_t motors = {};
        coreTask.run(state, demands, motors);

        // Set motor velocities from spin magnitudes, with +/- direction for
        // visual effect
        wb_motor_set_velocity(_m1_motor, +motors.m1);
        wb_motor_set_velocity(_m2_motor, -motors.m2);
        wb_motor_set_velocity(_m3_motor, +motors.m3);
        wb_motor_set_velocity(_m4_motor, -motors.m4);
    }

    wb_robot_cleanup();

    return 0;
}
