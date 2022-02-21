# Led 0 (Charging)

| Color           | State                         |
| --------------------- | ------------------------------ |
| Off                   | Charging disabled              |
| Red (solid)           | Not charging                   |
| Blue (solid)          | Charging (charge amps > thresh |
| Yellow (solid)        | Calibration                    |

Additional work planned in https://github.com/pats-drones/pats/issues/1177


# Led 1 (system)

| Color           | State                         |
| --------------------- | ----------------------------- |
| Red (solid)           | LED1_init                     |
| Red (blink 0.5s)      | LED1_inresponsive_NUC         |
| Red (blink 1.0s)      | LED1_internal_system_error    |
| pink (solid)          | LED1_realsense_reset          |
| White (solid)         | LED1_executor_start           |
| blue (solid)          | LED1_wait_for_plukkers        |
| yellow (solid)        | LED1_wait_for_darkness        |
| green (solid)         | LED1_c_OK                     |
| cyan (solid)          | LED1_x_OK                     |
| OFF                   | No Power to system?           |


| Bonus blinks            | State                         |
| ----------------------------- | ----------------------------- |
| blink_1_short_blink_reversed  | post_processing               |
| blink_2_short_blinks          | daemon problem                |
| blink_3_short_blinks          | internet (tunnel) problem     |
