{--
  X/Y position PID control algorithm for real and simulated flight controllers
 
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

module Position where

import Language.Copilot
import Copilot.Compile.C99

import Pid
import Demands
import State
import Utils

run kp ki dt ilimit target actual integ = (demand, integ') where

  error = target - actual

  demand = (-(kp * error + ki * integ))

  integ' = constrain (integ + error * dt) (-ilimit) (ilimit)

{--
  Demand is input as desired speed in meter per second, output as
  angles in degrees.
--}

positionPid dt state demands = demands'  where

  kp = 25
  ki = 1
  ilimit = 5000
    
  (rollDemand, rollInteg) = 
    run kp ki dt ilimit (roll demands) (dy state) rollInteg'

  rollInteg' = [0] ++ rollInteg

  (pitchDemand, pitchInteg) = 
    run kp ki dt ilimit (pitch demands) (dx state) pitchInteg'

  pitchInteg' = [0] ++ pitchInteg

  demands' = Demands (thrust demands) rollDemand pitchDemand (yaw demands)
