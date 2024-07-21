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

status_landed     = 0 :: SInt8
status_taking_off = 1 :: SInt8
status_flying     = 2 :: SInt8

-- We consider altitudes below this value to be the ground
zground = 0.05 :: SFloat

initial_altitude_target = 0.2 :: SFloat

-- We consider throttle inputs above this below this value to be positive
-- for takeoff
throttle_zero = 0.05 :: SFloat

throttle_scale = 0.005 :: SFloat

demandsStruct :: Stream DemandsStruct
demandsStruct = extern "stream_stickDemands" Nothing

stateStruct :: Stream StateStruct
stateStruct = extern "stream_vehicleState" Nothing

step = (motors, stickDemands) where

  state = liftState stateStruct

  stickDemands = liftDemands demandsStruct

  dt = rateToPeriod clock_rate

  altitude_target = if status == status_flying 
                    then  altitude_target + throttle_scale * 
                      (thrust stickDemands)
                    else if status == status_landed 
                    then initial_altitude_target 
                    else altitude_target'

  status = if status == status_taking_off  && (zz state) > zground
           then status_flying 
           else if status' == status_flying && (zz state) <= zground 
           then status_landed 
           else if status' == status_landed && 
                (thrust stickDemands) > throttle_zero 
           then status_taking_off
           else status'

  landed = status == status_landed

  status' = [0] ++ status

  altitude_target' = [0] ++ altitude_target

  demands = stickDemands

  demands' = altitudePid state dt altitude_target demands

  {--
  pids = [positionPid resetPids dt,
          pitchRollAnglePid resetPids dt,
          pitchRollRatePid resetPids dt,
          -- altitudePid dt,
          climbRatePid inFlyingMode dt,
          yawAnglePid dt,
          yawRatePid dt]

  demands' = foldl (\demand pid -> pid vehicleState demand) stickDemands pids

  thrust'' = if inFlyingMode then ((thrust demands') * tscale + tbase) else tmin
  --}


  thrust'' = thrust stickDemands

  motors = quadXMixer $ Demands thrust''
                                ((roll demands') * prscale)
                                ((pitch demands') * prscale)
                                ((yaw demands') * yscale)

------------------------------------------------------------------------------
 
spec = do

    let (motors, demands) = step

    let (me_ne, m_se, m_sw, m_nw) = motors

    trigger "setMotors" true [arg $ me_ne, arg $ m_se, arg $ m_sw, arg $ m_nw] 

-- Compile the spec
main = reify spec >>= 
  compileWith (CSettings "copilot_step_core" ".") "copilot_core"
