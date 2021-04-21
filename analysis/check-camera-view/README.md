# Check camera view

Check camera view definition (plane location) and check whether points are in or outside of the camera view.

## Usage
- Print plane parameter by setting `#define CAMERA_VIEW_DEBUGGING true` in `base/src/cameraview.h` and past them into `volume_log.txt`
- `main.py` prints the camera view and the location of `drone_location` (variable in main.py).
- `main.py` uses the function `is_pointInVolume(drone_state[0], planes)` also whether `drone_state[0]` (aka `drone_location`) is in the camera view. 

