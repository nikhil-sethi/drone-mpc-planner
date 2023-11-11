# Flight-Benchmark

The scripts gather all flights in which the dronenavigation-statemachine reached the state ns_takeing_off. This does not necessarily mean that the drone actually took off.

## Criteria overview

- In the exported csv-table for all criteria with multiple measurements the __max__-value is take.

## Crash criteria

The drone is considered as crashed, if:
- The dronenavigation-statemachine has not entered the state landed.
- The blink-location before and after the flight has an offset in y larger than 0.05 m.

That means that a drone that falls off the landing platform after the landing and which is not in view for a second blink location, is considered as _not crashed_. 
