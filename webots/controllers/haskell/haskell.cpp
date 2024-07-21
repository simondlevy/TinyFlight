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

#include <sim/quadcopter.hpp>

static Quadcopter _quadcopter;

// These are global so they can be shared with Haskell Copilot ---------------

demands_t stream_stickDemands;

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
    _quadcopter.setMotors(m1, m2, m3, m4);
}

// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
    stream_inFlyingMode = true;

    stream_resetPids = false;

    while (_quadcopter.step(stream_vehicleState, stream_stickDemands)) {

    }

    return 0;
}

void debugDemands(const float roll, const float pitch)
{
}
