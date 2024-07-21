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

int main(int argc, char ** argv)
{
    stream_inFlyingMode = false;

    stream_resetPids = false;

    while (wb_robot_step(timestep) != -1) {

        //Un-comment if you want to try OpenCV
        // runCamera(camera);

        // Get open-loop demands from input device (keyboard, joystick, etc.)
        float throttle = 0;
        _sticks.read(
                throttle,
                stream_openLoopDemands.roll, 
                stream_openLoopDemands.pitch, 
                stream_openLoopDemands.yaw,
                stream_inFlyingMode);

        stream_openLoopDemands.thrust = throttle;

        // Adjust roll for positive leftward
        stream_openLoopDemands.roll = -stream_openLoopDemands.roll;

        // Get vehicle state from sensors
        _getVehicleState(gyro, imu, gps);

        // Integrate stick demand to get altitude target
        altitudeTarget = _constrain(
                altitudeTarget + stream_openLoopDemands.thrust * DT, 
                ALTITUDE_TARGET_MIN, ALTITUDE_TARGET_MAX);

        // Rescale altitude target to [-1,+1]
        stream_openLoopDemands.thrust = 2 * ((altitudeTarget - ALTITUDE_TARGET_MIN) /
                (ALTITUDE_TARGET_MAX - ALTITUDE_TARGET_MIN)) - 1;

       copilot_step_core();

        //report(sec_start);
    }

    wb_robot_cleanup();

    return 0;
}

void debugDemands(const float roll, const float pitch)
{
}
