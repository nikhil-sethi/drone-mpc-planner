# Led 0 (drone)

| Color                 | Blink        | State                           |
| --------------------- | ------------ | --------------------------------|
| Off                   | solid        | Charging disabled               |
| Red                   | solid        | Not charging                    |
| Blue                  | solid        | Charging (charge amps > thresh) |
| Yellow                | solid        | Calibration                     |

| Bonus blinks                  | State                         |
| ----------------------------- | ----------------------------- |
| blink_1_short_blink           | rc problem                    |
| blink_2_short_blinks          | att calibration problem       |
| blink_3_short_blinks          | arm problem                   |


Additional work planned in https://github.com/pats-drones/pats/issues/1177


# Led 1 (system)

| Color                 | Blink         | State                         |
| --------------------- | ------------- | ----------------------------- |
| Red                   | solid         | LED1_init                     |
| Red                   | 1s symmetric  | LED1_inresponsive_NUC         |
| Red                   | 1 short blink | LED1_executor_problem         |
| Pink                  | solid         | LED1_realsense_reset          |
| White                 | solid         | LED1_executor_start           |
| Blue                  | solid         | LED1_wait_for_plukkers        |
| Yellow                | solid         | LED1_wait_for_darkness        |
| Green                 | solid         | LED1_c_OK                     |
| Cyan                  | solid         | LED1_x_OK                     |
| Off                   | solid         | No Power to system?           |


| Bonus blinks                  | State                         |
| ----------------------------- | ----------------------------- |
| blink_1_short_blink_reversed  | post_processing               |
| blink_2_short_blinks          | daemon problem                |
| blink_3_short_blinks          | internet (tunnel) problem     |
