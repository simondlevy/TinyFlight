/**
 * Haskell Copilot simulator support for Webots
 *
 * Copyright (C) 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <datatypes.h>
#include <hwebots.hpp>

// ----------------------------------------------------------------------------

// https://www.bitcraze.io/documentation/tutorials/getting-started-with-flow-deck/
static const float ALTITUDE_TARGET_INITIAL = 0.4;
static const float ALTITUDE_TARGET_MIN = 0.2;
static const float ALTITUDE_TARGET_MAX = 2.0;  // 3.0 in original

static const float DT = .01;

static Quadcopter _sim;

static WbDeviceTag _m1_motor;
static WbDeviceTag _m2_motor;
static WbDeviceTag _m3_motor;
static WbDeviceTag _m4_motor;


// These are global so they can be shared with Haskell Copilot ---------------

demands_t stream_openLoopDemands;

state_t stream_vehicleState;

bool stream_inFlyingMode;

bool stream_resetPids;

void report(float value)
{
    printf("%f\n", value);
}

void copilot_step_core(void);

void setMotors(float m1, float m2, float m3, float m4)
{
    // Set simulated motor values
    wb_motor_set_velocity(_m1_motor, +m1);
    wb_motor_set_velocity(_m2_motor, -m2);
    wb_motor_set_velocity(_m3_motor, +m3);
    wb_motor_set_velocity(_m4_motor, -m4);
}

// ---------------------------------------------------------------------------

static float _rad2deg(const float rad)
{
    return rad / M_PI * 180;
}

static void _getVehicleState(
        WbDeviceTag & gyro, 
        WbDeviceTag & imu, 
        WbDeviceTag & gps,
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

    // Get state values (meters, degrees) from ground truth:
    //   x: positive forward
    //   y: positive leftward
    //   z: positive upward
    //   phi, dphi: positive roll right
    //   theta,dtheta: positive nose up (requires negating imu, gyro)
    //   psi,dpsi: positive nose left
    state.z =        wb_gps_get_values(gps)[2];
    state.phi =     _rad2deg(wb_inertial_unit_get_roll_pitch_yaw(imu)[0]);
    state.dphi =    _rad2deg(wb_gyro_get_values(gyro)[0]);
    state.theta =  -_rad2deg(wb_inertial_unit_get_roll_pitch_yaw(imu)[1]);
    state.dtheta = -_rad2deg(wb_gyro_get_values(gyro)[1]); 
    state.psi =     _rad2deg(psi);
    state.dpsi =    _rad2deg(wb_gyro_get_values(gyro)[2]);

    // Use temporal first difference to get world-cooredinate velocities
    auto x = wb_gps_get_values(gps)[0];
    auto y = wb_gps_get_values(gps)[1];
    auto dx = (x - xprev) / dt;
    auto dy = (y - yprev) / dt;
    state.dz = (state.z - zprev) / dt;

    // Rotate X,Y world velocities into body frame to simulate optical-flow
    // sensor
    auto cospsi = cos(psi);
    auto sinpsi = sin(psi);
    state.dx = dx * cospsi + dy * sinpsi;
    state.dy = dy * cospsi - dx * sinpsi;

    // Save past time and position for next time step
    xprev = x;
    yprev = y;
    zprev = state.z;
}

static float _constrain(const float val, const float lo, const float hi)
{
    return val < lo ? lo : val > hi ? hi : val;
}

int main(int argc, char ** argv)
{
    wb_robot_init();

    const int timestep = (int)wb_robot_get_basic_time_step();

    // Initialize motors
    _m1_motor = Quadcopter::makeMotor("m1_motor", +1);
    _m2_motor = Quadcopter::makeMotor("m2_motor", -1);
    _m3_motor = Quadcopter::makeMotor("m3_motor", +1);
    _m4_motor = Quadcopter::makeMotor("m4_motor", -1);

    // Initialize sensors
    auto imu = Quadcopter::makeSensor("inertial_unit", timestep, wb_inertial_unit_enable);
    auto gyro = Quadcopter::makeSensor("gyro", timestep, wb_gyro_enable);
    auto gps = Quadcopter::makeSensor("gps", timestep, wb_gps_enable);
    auto camera = Quadcopter::makeSensor("camera", timestep, wb_camera_enable);

    _sim.initSticks();

    float altitudeTarget = ALTITUDE_TARGET_INITIAL;

    stream_inFlyingMode = true;
    stream_resetPids = false;

    while (wb_robot_step(timestep) != -1) {

        //Un-comment if you want to try OpenCV
        // runCamera(camera);

        // Get open-loop demands from input device (keyboard, joystick, etc.)
        float throttle = 0;
        _sim.readSticks(
                throttle,
                stream_openLoopDemands.roll, 
                stream_openLoopDemands.pitch, 
                stream_openLoopDemands.yaw);

        stream_openLoopDemands.thrust = throttle;

        // Adjust roll for positive leftward
        stream_openLoopDemands.roll = -stream_openLoopDemands.roll;

        // Get vehicle state from sensors
        _getVehicleState(gyro, imu, gps, stream_vehicleState);

        // Integrate stick demand to get altitude target
        altitudeTarget = _constrain(
                altitudeTarget + stream_openLoopDemands.thrust * DT, 
                ALTITUDE_TARGET_MIN, ALTITUDE_TARGET_MAX);

        // Rescale altitude target to [-1,+1]
        stream_openLoopDemands.thrust = 2 * ((altitudeTarget - ALTITUDE_TARGET_MIN) /
                (ALTITUDE_TARGET_MAX - ALTITUDE_TARGET_MIN)) - 1;

       copilot_step_core();
    }

    wb_robot_cleanup();

    return 0;
}

void debugDemands(const float roll, const float pitch)
{
}
