## Running the simulation

- Run the simulation with the `simulation_launch.py` file. It will load a map in `map_simulator` with some beacons.
The `ekf.yaml` configuration file is updated with the generated beacon positions.

- Spawn a robot with `spawn_launch.py`, ranges are published on the `ranges` topic.

## Example

A basic trilateration node can be run with `trilat_launch.py`. It will subscribe to the ranges and tf in order to update the `map -> odom` pose for the robot, using only the `(x,y)` position.
