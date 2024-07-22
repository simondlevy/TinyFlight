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

#include <datatypes.h>
#include <utils.hpp>
#include <webots.hpp>

// ----------------------------------------------------------------------------

// https://www.bitcraze.io/documentation/tutorials/getting-started-with-flow-deck/
static const float ALTITUDE_TARGET_INITIAL = 0.4;
static const float ALTITUDE_TARGET_MIN = 0.2;
static const float ALTITUDE_TARGET_MAX = 2.0;  // 3.0 in original

static const float DT = .01;

// Webots helper class
static Quadcopter _sim;

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
    _sim.setMotors(m1, m2, m3, m4);
}

// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
    _sim.init();

    float altitudeTarget = ALTITUDE_TARGET_INITIAL;

    stream_inFlyingMode = true;
    stream_resetPids = false;

    while (_sim.isRunning()) {

        //Un-comment if you want to try OpenCV
        // runCamera(camera);

        // Get open-loop demands from input device (keyboard, joystick, etc.)
        _sim.readSticks(
                stream_openLoopDemands.thrust,
                stream_openLoopDemands.roll, 
                stream_openLoopDemands.pitch, 
                stream_openLoopDemands.yaw);

        // Get vehicle state from sensors
        _sim.getVehicleState(stream_vehicleState);

        // XXX
        stream_vehicleState.theta *= -1;
        stream_vehicleState.dtheta *= -1;
        //stream_vehicleState.dy *= -1;

        // Integrate stick demand to get altitude target
        altitudeTarget = Utils::fconstrain(
                altitudeTarget + stream_openLoopDemands.thrust * DT, 
                ALTITUDE_TARGET_MIN, ALTITUDE_TARGET_MAX);

        // Rescale altitude target to [-1,+1]
        stream_openLoopDemands.thrust = 
            2 * ((altitudeTarget - ALTITUDE_TARGET_MIN) /
                (ALTITUDE_TARGET_MAX - ALTITUDE_TARGET_MIN)) - 1;

       copilot_step_core();
    }

    _sim.close();

    return 0;
}

