{--
  LambdaFlight core algorithm: reads open-loop demands and
  state as streams; runs PID controllers and motor mixers
 
  Copyright (C) 2024 Simon D. Levy
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, in version 3.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
--} 

{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module CoreTask where

import Language.Copilot
import Copilot.Compile.C99

import Clock
import Constants
import Demands
import Mixers
import Sensors
import State
import Utils

import Constants

-- PID controllers
import Altitude
import ClimbRate
import PitchRollAngle
import PitchRollRate
import Position
import YawAngle
import YawRate

-- Streams from C++ ----------------------------------------------------------

demandsStruct :: Stream DemandsStruct
demandsStruct = extern "stream_stickDemands" Nothing

stateStruct :: Stream StateStruct
stateStruct = extern "stream_vehicleState" Nothing

altitudeTarget :: SFloat
altitudeTarget = extern "stream_altitudeTarget" Nothing

landed :: SBool
landed = extern "stream_landed" Nothing

step = (motors, ydemand) where

  state = liftState stateStruct

  stickDemands = liftDemands demandsStruct

  dt = rateToPeriod clock_rate

  pids = [positionController dt,
          pitchRollAngleController dt,
          pitchRollRateController landed dt,
          altitudeController altitudeTarget dt,
          climbRateController (not landed) dt,
          yawAngleController dt,
          yawRateController dt]

  demands = foldl (
     \demand pid -> pid state demand) stickDemands pids

  ydemand = roll demands

  motors = runCF $ Demands (thrust demands)
                           (roll demands) 
                           (pitch demands)
                           (yaw demands)

------------------------------------------------------------------------------
 
spec = do

    let (motors, ydemand) = step

    let (m1, m2, m3, m4) = motors

    trigger "setMotors" true [arg $ m1, arg $ m2, arg $ m3, arg $ m4] 

    trigger "debug" true [arg ydemand]

-- Compile the spec
main = reify spec >>= 
  compileWith (CSettings "copilot_step_core" ".") "copilot_core"
