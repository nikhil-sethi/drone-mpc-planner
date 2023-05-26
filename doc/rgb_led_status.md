# Led 0 (drone)

| Color                 | Blink                       | State                           |
| --------------------- | --------------------------- | --------------------------------|
| Off                   | solid                       | Charging disabled               |
| Red                   | solid                       | Not charging                    |
| Blue                  | solid                       | Charging (charge amps > thresh) |
| Cyan                  | solid                       | Trickle charging                |
| Blue                  | blink_3_short_blinks        | Discharging                     |
| Yellow                | blink_3_short_blinks        | Battery problem                 |
| Orange                | 1s symmetric                | RC telemetry time out           |
| Orange                | blink_1_short_blinks        | Blink locate fail               |
| White                 | 1s symmetric                | Crashed                         |


# Led 1 (system)

| Color                 | Blink         | State                         |
| --------------------- | ------------- | ----------------------------- |
| Red                   | solid         | LED1_init                     |
| Red                   | 1s symmetric  | LED1_inresponsive_NUC         |
| Red                   | 1 short blink | LED1_executor_problem         |
| Pink                  | solid         | LED1_realsense_reset          |
| White                 | solid         | LED1_executor_start           |
| Blue                  | solid         | LED1_wait_for_enable_window   |
| Yellow                | solid         | LED1_wait_for_lightlevel      |
| Green                 | solid         | LED1_c_OK                     |
| Cyan                  | solid         | LED1_x_OK                     |
| Orange                | solid         | LED1_x_READY                  |
| White                 | solid         | LED1_blind_OK                 |
| Off                   | solid         | No Power to system?           |


| Bonus blinks                  | State                         |
| ----------------------------- | ----------------------------- |
| blink_1_short_blink_reversed  | post_processing               |
| blink_2_short_blinks          | daemon problem                |
| blink_3_short_blinks          | internet (tunnel) problem     |
