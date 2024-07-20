# TinyFlight
Minimalist flight-control firmware with simulator


# Webots

Install [Webots](https://cyberbotics.com/)

```
cd webots/controllers/simple
make
```

Launch webots; Open World webots/worlds/simple.wbt

# TinyApe (not ready for general use)

```
cd examples/tinyape
make
make flash
```

# Axis conventions

To keep things simple and avoid sign changes in the PID controllers, the
following state values and demands directions are all positive:

* Roll right
* Move right (DY)
* Pitch forward
* Move forward (DX)
* Nose right 
* Climb (Z, DZ)
