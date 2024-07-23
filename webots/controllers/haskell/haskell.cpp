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
static const float ALTITUDE_TARGET_INITIAL = 0.2;
static const float ALTITUDE_TARGET_MIN = 0.2;
static const float ALTITUDE_TARGET_MAX = 2.0;

static const float DT = .01;

// Webots helper class
static Quadcopter _sim;

// These are global so they can be shared with Haskell Copilot ---------------

demands_t stream_stickDemands;

state_t stream_vehicleState;

float stream_altitudeTarget;

bool stream_landed;

void debug(float value)
{
    printf("%+3.3f\n", value);
}

void copilot_step_core(void);

void setMotors(float m1, float m2, float m3, float m4)
{
    _sim.setMotors(m1, m2, m3, m4);
}

// ---------------------------------------------------------------------------

enum {

    STATUS_LANDED,
    STATUS_TAKING_OFF,
    STATUS_FLYING

};

int main(int argc, char ** argv)
{
    _sim.init();

    stream_altitudeTarget = 0;

    uint8_t status = STATUS_LANDED;

    static const float THROTTLE_ZERO = 0.05;
    static const float THROTTLE_SCALE = 0.005;
    static const float ZGROUND = 0.05;

    while (_sim.isRunning()) {

        // Get open-loop demands from input device (keyboard, joystick, etc.)
        _sim.readSticks(
                stream_stickDemands.thrust,
                stream_stickDemands.roll, 
                stream_stickDemands.pitch, 
                stream_stickDemands.yaw);

        // Get vehicle state from sensors
        _sim.getVehicleState(stream_vehicleState);

        stream_altitudeTarget =
            status == STATUS_FLYING ? 
            stream_altitudeTarget + THROTTLE_SCALE *
              stream_stickDemands.thrust :
            status == STATUS_LANDED ?
            ALTITUDE_TARGET_INITIAL :
            stream_altitudeTarget;

        status = 

            status == STATUS_TAKING_OFF  && stream_vehicleState.z > ZGROUND ?  
            STATUS_FLYING :

            status == STATUS_FLYING && stream_vehicleState.z <= ZGROUND ?  
            STATUS_LANDED :

            status == STATUS_LANDED && 
            stream_stickDemands.thrust > THROTTLE_ZERO ? 
            STATUS_TAKING_OFF :

            status;

        stream_landed = status == STATUS_LANDED;

        copilot_step_core();
    }

    _sim.close();

    return 0;
}

