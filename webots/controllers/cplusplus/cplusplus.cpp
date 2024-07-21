#include <sim/quadcopter.hpp>

int main(int argc, char ** argv)
{
    static Quadcopter quadcopter;

    quadcopter.run();

    return 0;
}
