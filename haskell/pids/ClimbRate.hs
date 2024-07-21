{--
  Climb-rate algorithm for real and simulated flight controllers
 
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

module ClimbRate where

import Language.Copilot
import Copilot.Compile.C99

import Pid
import Demands
import State
import Utils

run dt thrust dz = thrust'  where

    kp = 25
    ki = 15
    ilimit = 5000

    (thrust', integ) = piController kp ki dt ilimit thrust dz integ'

    integ' = [0] ++ integ


{-- 

  Demand is input as climb rate in meters per second and output as arbitrary
  positive value to be scaled according to motor characteristics.

--}

climbRatePid state dt tbase tscale tmin flying demands = demands' where

    thrustpid = run dt (thrust demands) (dz state)

    thrustout = if flying then thrustpid * tscale + tbase else tmin

    demands' = Demands thrustout (roll demands) (pitch demands) (yaw demands)
