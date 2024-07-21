#pragma once

#include <time.h>

#include <webots/camera.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <datatypes.h>
#include <utils.hpp>

#include <sim/sticks.hpp>

class Quadcopter {

    public:

        Quadcopter(void)
        {
            wb_robot_init();

            _timestep = (int)wb_robot_get_basic_time_step();

            // Initialize motors
            _m1_motor = _makeMotor("m1_motor", +1);
            _m2_motor = _makeMotor("m2_motor", -1);
            _m3_motor = _makeMotor("m3_motor", +1);
            _m4_motor = _makeMotor("m4_motor", -1);

            // Initialize sensors
            _imu =
                _makeSensor("inertial_unit", _timestep, wb_inertial_unit_enable);
            _gyro = _makeSensor("gyro", _timestep, wb_gyro_enable);
            _gps = _makeSensor("gps", _timestep, wb_gps_enable);
            _camera = _makeSensor("camera", _timestep, wb_camera_enable);

            _sticks.init();
        }

        ~Quadcopter(void)
        {
            wb_robot_cleanup();
        }

        bool step(state_t & state, demands_t & demands)
        {
            if (wb_robot_step(_timestep) == -1) {
                return false;
            }

            //Un-comment if you want to try OpenCV
            // runCamera(camera);

            // Get open-loop demands from input device (keyboard, joystick,
            // etc.)
            _sticks.read(
                    demands.thrust, demands.roll, demands.pitch, demands.yaw);

            // Get vehicle state from sensors
            getVehicleState(state);

            return true;
        }

        void setMotors(
                const float m1, const float m2, const float m3, const float m4)
        {
            wb_motor_set_velocity(_m1_motor, +m1);
            wb_motor_set_velocity(_m2_motor, -m2);
            wb_motor_set_velocity(_m3_motor, +m3);
            wb_motor_set_velocity(_m4_motor, -m4);
        }

        void getVehicleState(state_t & state)
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
            auto psi = wb_inertial_unit_get_roll_pitch_yaw(_imu)[2];

            // Get state variables, negating gyro for nose-right positive
            state.z  = wb_gps_get_values(_gps)[2];
            state.phi = 
                Utils::RAD2DEG*(wb_inertial_unit_get_roll_pitch_yaw(_imu)[0]);
            state.dphi = 
                Utils::RAD2DEG*(wb_gyro_get_values(_gyro)[0]);
            state.theta = 
                Utils::RAD2DEG*(wb_inertial_unit_get_roll_pitch_yaw(_imu)[1]);

            state.dtheta =  Utils::RAD2DEG*(wb_gyro_get_values(_gyro)[1]); 
            state.psi  =  -Utils::RAD2DEG*(psi); 
            state.dpsi =  -Utils::RAD2DEG*(wb_gyro_get_values(_gyro)[2]);

            // Use temporal first difference to get world-cooredinate velocities
            auto x = wb_gps_get_values(_gps)[0];
            auto y = wb_gps_get_values(_gps)[1];
            auto dx = (x - xprev) / dt;
            auto dy = (y - yprev) / dt;
            state.dz = (state.z - zprev) / dt;

            // Rotate X,Y world velocities into body frame to simulate
            // optical-flow sensor
            auto cospsi = cos(psi);
            auto sinpsi = sin(psi);
            state.dx = dx * cospsi + dy * sinpsi;
            state.dy = dx * sinpsi - dy * cospsi;

            // Save past time and position for next time step
            xprev = x;
            yprev = y;
            zprev = state.z;
        }
    private:

        // https://www.bitcraze.io/documentation/tutorials/
        //   getting-started-with-flow-deck/
        static constexpr float ALTITUDE_TARGET_INITIAL = 0.4;
        static constexpr float ALTITUDE_TARGET_MIN = 0.2;
        static constexpr float ALTITUDE_TARGET_MAX = 2.0;  // 3.0 in original

        int _timestep;

        // Motors
        WbDeviceTag _m1_motor;
        WbDeviceTag _m2_motor;
        WbDeviceTag _m3_motor;
        WbDeviceTag _m4_motor;

        // Sensors
        WbDeviceTag _imu;
        WbDeviceTag _gps;
        WbDeviceTag _gyro;
        WbDeviceTag _camera;

        Sticks _sticks;

        static WbDeviceTag _makeMotor(const char * name, const float direction)
        {
            auto motor = wb_robot_get_device(name);

            wb_motor_set_position(motor, INFINITY);
            wb_motor_set_velocity(motor, direction);

            return motor;
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

};

