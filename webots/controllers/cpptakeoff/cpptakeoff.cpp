#include <webots/camera.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/gps.h>

// Control constants
static const float K1 = 56;
static const float K2 = 25;
static const float K3 = 2;

static const float ZTARGET = 0.2;

static const float DT = 0.01;

static const float PLOT_TIME_LIMIT = 7;

static const float SPINUP_TIME = 1.5;

static const float SPIN_SCALEUP = 2;

// Motors
static WbDeviceTag _m1_motor;
static WbDeviceTag _m2_motor;
static WbDeviceTag _m3_motor;
static WbDeviceTag _m4_motor;

static WbDeviceTag _makeMotor(const char * name, const float direction)
{
    auto motor = wb_robot_get_device(name);

    wb_motor_set_position(motor, INFINITY);
    wb_motor_set_velocity(motor, direction);

    return motor;
}

int main(int argc, char ** argv)
{
    wb_robot_init();

    _m1_motor = _makeMotor("m1_motor", +1);
    _m2_motor = _makeMotor("m2_motor", -1);
    _m3_motor = _makeMotor("m3_motor", +1);
    _m4_motor = _makeMotor("m4_motor", -1);

    const int timestep = (int)wb_robot_get_basic_time_step();

    auto gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, timestep);

    // Start with faked-up initial motor to animate motor spin-up
    auto motor = K2 + K3;

    float zprev = 0;

    while (wb_robot_step(timestep) != -1) {

        // Get current altitude
        const auto z = wb_gps_get_values(gps)[2];
        
        const auto motor =
            K1 + K2 * (K3 * (ZTARGET - z) - ((z - zprev) / DT));

        zprev = z;

        // Animate the motors
        wb_motor_set_velocity(_m1_motor, +motor);
        wb_motor_set_velocity(_m2_motor, -motor);
        wb_motor_set_velocity(_m3_motor, +motor);
        wb_motor_set_velocity(_m4_motor, -motor);
    }

    wb_robot_cleanup();

    return 0;
}
