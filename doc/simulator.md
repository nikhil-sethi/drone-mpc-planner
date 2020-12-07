## Simulator
A high fidelity simulator to increase testing and validation speed.

## Table of contents

- [Simulator](#simulator)
- [Table of contents](#table-of-contents)
- [Getting Started](#getting-started)
  - [Installation](#installation)
  - [Quick start guide](#quick-start-guide)
  - [Starting the simulator](#starting-the-simulator)
  - [Updating the simulator](#updating-the-simulator)
    - [User version](#user-version)
    - [Dev version](#dev-version)
- [Unreal](#unreal)
  - [Using the editor](#using-the-editor)
    - [Shortcuts](#shortcuts)
  - [Customizing the drone](#customizing-the-drone)
    - [Visuals](#visuals)
  - [Adjusting the baseline](#adjusting-the-baseline)
    - [Parameters](#parameters)
  - [Lighting](#lighting)
  - [Adding STEP files](#adding-step-files)
  - [Compiling to binaries](#compiling-to-binaries)
  - [Virtual Moth](#virtual-moth)
  - [Dimensions](#dimensions)
- [AirSim](#airsim)
  - [Making changes to AirSim](#making-changes-to-airsim)
  - [Settings](#settings)
    - [User version](#user-version-1)
    - [Dev version](#dev-version-1)
  - [ViewMode](#viewmode)
  - [Images](#images)
    - [Converting FOV to focal length](#converting-fov-to-focal-length)
    - [Grayscale images from `simGetImages`](#grayscale-images-from-simgetimages)
  - [Creating a new environment with AirSim](#creating-a-new-environment-with-airsim)
  - [Creating a new level](#creating-a-new-level)
  - [Updating AirSim](#updating-airsim)
  - [Betaflight software-in-the-loop](#betaflight-software-in-the-loop)
- [Troubleshooting](#troubleshooting)
- [Examples](#examples)
  - [moveByRC](#movebyrc)

## Getting Started

### Installation

Start `install_sim.sh` to install all requirements.

There are two versions of the simulator available for install.

1. **User version** for using the simulator. Run the install script with argument 0.
1. **Dev version** for developing the simulator. Run the install script with argument 1.

> **Note** the dev installation script will clone and compile the Unreal Engine. This will take some time.

If you have installed the user version, please see the [Quick start guide](#quick-start-guide). For the developer version, continue at [Starting the simulator](#starting-the-simulator).

### Quick start guide

Run `startsim` in the terminal. This will open a black window, the simulation is now running. With `alt+tab` you get your mouse back. Now you can start the Pats software with the `--airsim` flag and the virtual environment should appear. Use the `--airsim-wp` flag to also automatically start a waypoint flight. If something went wrong during this process, please look at the [Troubleshooting](#troubleshooting) guide. To change the map that the simulator is loading, please use `--airsim <mapname>` or `--airsim-wp <mapname>`. The maps currently available are:

1. Greenhouse (default)
2. GreenhouseEmpty
3. GreenhouseTomato

### Starting the simulator

Run `startsimdev` in the terminal. This will open the Greenhouse environment in the Unreal Editor. If you are prompted for `Missing Modules` and asked `Would you like to rebuild them now?`, choose `Yes`. When loading is complete, click the build button. When building is complete, press play and the simulator will run in the black window. Now run the pats software with the `--airsim` flag and the virtual environment should appear. Use the `--airsim-wp` flag to also utomatically start a waypoint flight.

**Important:** open the editor and go to "Edit -> Editor Preferences" in the "Search" box, type "CPU" and ensure that the `Use Less CPU when in Background` is **unchecked**.

### Updating the simulator

#### User version
To update the user version of the simulator, just run the install script again with argument 0.

#### Dev version

To update the dev version, we have 4 different components to update.

1. **AirSimEnvironment** - `git pull`
2. **AirSim** - `git pull && ./build.sh`
3. **Pats** - `git checkout simulator && git pull`
4. **Unreal Engine** - First check if updating the UE is necessary [here](https://microsoft.github.io/AirSim/build_linux/#linux-build-unreal-engine). At the time of writing, UE 4.25 is recommended.

  ```bash
  git checkout 4.xx
  ./Setup.sh
  ./GenerateProjectFiles.sh
  make
  ```

## Unreal

### Using the editor

#### Shortcuts

Use Fn+F1 to display the following list of commands
> The simulation automatically resets when you close the Pats window

```
F1          Toggle this help
F3          Toggle wireframe whe you press F1
F10         Show weather options
F           Switch to FPV View
B           Switch to "fly with me"
\           Switch to ground observer view
/           Switch to chase with spring arm mode
M           Switch to manual camera control
                    Arrow keys
                    Page Up/Down -> move up, down
                    W, S -> pitch
                    A, D -> yaw
R           Toggle recording
;           Toggle debug report
0           Toggle all sub-windows
1           Toggle depth sub-window
2           Toggle segmentation sub-window
3           Toggle scene sub-window
T           Toggle trace line
Backspace   Reset everything
```

### Customizing the drone

#### Visuals

To change the drone blueprint, the following steps are necessary

- The Drone Actor is located at `Content/Models/BP_FlyingPawnHammer`. Open this blueprint.
- Here you can change the mesh, add components or scale the drone.
- When you are finished adjusting the drone, click on compile and save and the drone is modified.

### Adjusting the baseline

To adjust the baseline of the drone we need to adjust the blueprint. If you don't know how to do that look at [Visuals](#visuals). Under the bodyMesh in the Components panel is the `FrontRightCamera` and the `FrontLeftCamera`. The Y coordinate must be the baseline divided by two, as it is calculated from the center for both cameras. So for a baseline of 9cm the y-coordinate of one camera must be adjusted to -4.5 and of the other camera to 4.5.

#### Parameters

To customize the drone behaviour, it is necessary to make changes to the code of AirSim. The following files are important:

- `AirLib/include/vehicles/multirotor/RotorParams.hpp`
- `AirLib/include/vehicles/multirotor/MultiRotorParams.hpp`
- `AirLib/include/vehicles/multirotor/firmwares/simple_flight/firmware/Params.hpp`

After changing the parameters, run the `./build.sh` script in the root folder of AirSim. This will automatically update the Greenhouse environment.

### Lighting

The main light source is the `SkyLight` found in the "World Outliner" panel in the editor. Use the Intensity scale option to in or decrease the lighting condition.

### Adding STEP files

Export the 3D model as a `.STEP` file. To import STEP files into Unreal we use the Datasmith plugin. At the time of writing [STEP files are not supported on Linux](https://docs.unrealengine.com/en-US/Engine/Content/Importing/Datasmith/Platforms/index.html), so you need a windows installation with the Unreal Engine for this.

1. Enable the [Datasmith plugin](https://docs.unrealengine.com/en-US/Engine/Content/Importing/Datasmith/HowTo/ImportingContent/index.html#projectsetup)
2. In the main toolbar, click the Datasmith button:
3. Select the correct STEP file
4. Choose a folder in your Project content for Datasmith to put the newly imported assets.
5. Use the Datasmith Import Options window to select the types of content that you want to import from your source file, and set optional additional parameters that control the import process.
6. Now the model appears as a `.udatasmith` file. Now convert this format to a static mesh with `Actor Merging`, which can be found under experimental in the editor preferences.
7. Copy the generated `.uasset` file to the ubuntu machine in the `~/$DIRECTORY/AirSimEnvironment/Greenhouse/Content/Model/MODELNAME`
8. Now start the editor and drag the `.uasset` file in the environment

### Compiling to binaries

To make the use of the simulator much easier, we compile the simulator environment to binaries. Packaging is detailed [here](https://docs.unrealengine.com/en-US/Engine/Basics/Projects/Packaging/index.html) in the Unreal docs.

- Go to `File > Package Project > Packaging Settings..`
- Make sure that under "Additional Asset Directories to Cook" the `Game/Models/Drone` and `Game/Models/Basestation` directories are added. Please do not include `Game/Models`, this increases the package size.
- Make sure that under "List of maps to include in a packaged build", all the maps are included.
- Package for Ubuntu with `File > Package Project > Linux`
- Specifiy a destination folder (`AirSimEnvironment/Build`) and proceed

> **Note** the packaging process can be time consuming

The results is folder `AirSimEnvironment/Build` whose content can be launched with

```bash
Greenhouse.sh -ResX=640 -ResY=480 -windowed
```

To make it self-contained, move `settings.json` to the executable.

```bash
cp ~/code/AirSimEnvironment/settings.json ~/code/AirSimEnvironment/Build/LinuxNoEditor/Greenhouse/Binaries/Linux/
```

Now zip `Greenhouse.sh` together with the directories `Engine` and `Greenhouse` and the simulator is ready for release. The releases are located at the [AirSimEnvironment](https://github.com/pats-drones/AirSimEnvironment/releases) repository. Please use [semantic versioning](https://semver.org/) to label the releases and note which AirSim commit ID was used to compile this release.

### Virtual Moth

I have added a first version of a moth. This moth flies based on Unreal Blueprints. This is a visual scripting system built into the Unreal Engine and is a good way to prototype. At the moment, the moth can be dragged into the world, and then a point can be selected, along with a range. The model is located in `Models/Insect`. Now the moth is going to fly in random fashion between different spots in the range, with intermediate stops on the bushes for example. This behaviour can be changed when opening the blueprint located at `Models/Insect/Blueprints/BP_Butterfly`.

### Dimensions

In the Unreal Engine, you can set how much uu (the unit used in the engine) equals a cm. In the case of the virtual greenhouse, 1uu equals 1 cm.

## AirSim

### Making changes to AirSim

1. Make and save changes
2. run `./build.sh` in AirSim root
3. Restart the Unreal Editor, and when asked to rebuild click `Yes`
4. **Optional** when making changes to AirLib also re-compile pats code

### Settings

#### User version

The `settings.json` file for the compiled version is located at the executable path.

1. `~/path/to/Simulator/Greenhouse/Binaries/Linux/settings.json`
2. If the above path does not contain a settings file, then AirSim will look at `~/Documents/AirSim/settings.json`.

#### Dev version

The settings file is located at `~/$DIRECTORY/AirSimEnvironment/settings.json`. This file is linked to `~/Documents/AirSim/settings.json`, this is the location where AirSim reads it configuration file. Since JSON does not allow comments, I will explain here how our settings.json was created.

```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "ViewMode": "NoDisplay", // save resources by not rendering anyting in the viewport (see ViewMode for more information)
    "CameraDefaults": {
      "CaptureSettings": [
        {
          "ImageType": 0, // Scene = 0
          "Width": 848,
          "Height": 480,
          "AutoExposureSpeed": 100, // The AutoExposureSpeed decides how fast eye adaptation works. We set this to a high value such as 100 to avoid artifacts in image capture.
          "AutoExposureMaxBrightness": 0.64,
          "AutoExposureMinBrightness": 0.1,
          "FOV_Degrees": 90,
          "TargetGamma": 1.0
        },
        {
          "ImageType": 3, // DepthVis = 3
          "Width": 848,
          "Height": 480,
          "AutoExposureSpeed": 100,
          "AutoExposureMaxBrightness": 0.64,
          "AutoExposureMinBrightness": 0.03,
          "FOV_Degrees": 90,
          "TargetGamma": 1.0
        }
      ]
    },
    "PawnPaths": {
      // use custom pawns for the drone and basestation
      "DefaultQuadrotor": {"PawnBP": "Class'/Game/Models/Drone/BP_FlyingPawnHammer.BP_FlyingPawnHammer_C'"},
      "Basestation": {"PawnBP": "Class'/Game/Models/Basestation/BP_Basestation.BP_Basestation_C'"}
    },
    "OriginGeopoint": {
      "Latitude": 51.989899,
      "Longitude": 4.375370,
      "Altitude": 1
    },
    "TimeOfDay": {
      "Enabled": false,
      "StartDateTime": "2020-10-31 00:00:00",
      "CelestialClockSpeed": 1,
      "StartDateTimeDst": false,
      "UpdateIntervalSecs": 60
    },
    "Vehicles": {
      "Basestation": {
        "VehicleType": "SimpleFlight",
        "DefaultVehicleState": "Disarmed",
        "X": 3.3, "Y": 0.15, "Z": -3.5,
        "PawnPath": "Basestation", // use the pawnpath we set earlier
        "RC": {
          "RemoteControlID": -1 // disable USB RC since we are using the moveByRC command
        }
      },
      "Hammer": {
        "VehicleType": "SimpleFlight",
        "DefaultVehicleState": "Disarmed",
        "X": 0, "Y": 0, "Z": -1.55,
        "RC": {
          "RemoteControlID": -1 // disable USB RC since we are using the moveByRC command
        }
      }
    }
  }
```

### ViewMode

The ViewMode determines which camera to use as default and how camera will follow the vehicle. This is set to `NoDisplay` by default. You can also use shortcuts to chnage the `ViewMode`, see [Shortcuts](#shortcuts).

This setting can be found at the following line in `settings.json` mentioned previously

```json
"ViewMode": "NoDisplay",
```

The following options are available

1. `FlyWithMe`: Chase the vehicle from behind with 6 degrees of freedom
2. `GroundObserver`: Chase the vehicle from 6' above the ground but with full freedom in XY plane.
3. `Fpv`: View the scene from the basestation
4. `Manual`: Don't move camera automatically. Use arrow keys and ASWD keys for move camera manually.
5. `SpringArmChase`: Chase the vehicle with camera mounted on (invisible) arm that is attached to the vehicle via spring (so it has some latency in movement).
6. `NoDisplay`: This will freeze rendering for main screen, however rendering for subwindows, recording and APIs remain active. This mode is useful to save resources in "headless" mode where you are only interested in getting images and don't care about what gets rendered on main screen.

The camera settings applies to the first pawn in the settings file (under `Vehicles`). This is the Basestation by default, so `Fpv` will show the basestation view. If you want to chase the drone, then you need to move the drone to the first spot in the settings file. For `Manual`, the order of pawns does not matter.

### Images

#### Converting FOV to focal length

[Source](https://photo.stackexchange.com/questions/97213/finding-focal-length-from-image-size-and-fov)

```cpp
// converting FOV to focal length: f = (Width/2) / tan(FOV/2)
float focal_length = (IMG_W/2)/ tan((sim.getCameraFov() / 2) * ((float)M_PI/180));;
```

#### Grayscale images from `simGetImages`

To speed up the `simGetImages` request, I return the image as grayscale when using RGB uncompressed. This change can be found [here](https://github.com/pats-drones/AirSim/commit/7e2715f9b92df6086cf7de13aecdba28c6c53f3b) at the repository.

```cpp
for (unsigned int i = 0; i < req_size; ++i) {
        if (!params[i]->pixels_as_float) {
            if (results[i]->width != 0 && results[i]->height != 0) {
                if (params[i]->compress){
                    results[i]->image_data_uint8.SetNumUninitialized(results[i]->width * results[i]->height * 3, false);
                    UAirBlueprintLib::CompressImageArray(results[i]->width, results[i]->height, results[i]->bmp, results[i]->image_data_uint8);
                }
                else {
                    results[i]->image_data_uint8.SetNumUninitialized(results[i]->width * results[i]->height, false);
                    uint8* ptr = results[i]->image_data_uint8.GetData();
                    for (const auto& item : results[i]->bmp) {
                        *ptr++ = item.R;
                    }
                }
            }
        }
        else {
            results[i]->image_data_float.SetNumUninitialized(results[i]->width * results[i]->height);
            float* ptr = results[i]->image_data_float.GetData();
            for (const auto& item : results[i]->bmp_float) {
                *ptr++ = item.R.GetFloat();
            }
        }
    }
```

### Creating a new environment with AirSim

We use the Greenhouse environment I built for the simulator. If you want to create a new environment with the AirSim plugin, you have to do the following:

- Start the Unreal Editor with `$ UE4Editor` in your terminal
- On the "Select or Create New Project" menu, under "New Project Categories", choose "Games" and click "Next"
- On the "Select Template" menu, pick "Blank" and click "Next"
- On the "Project Settings" menu, open the dropdown menu of the "Blueprint" tile and choose "C++" instead
- On the same menu,  open the dropdown menu of the "With Starter Content" tile and choose "No Starter Content"
- On the same menu, pick a folder (e.g. `~/code/`) and name (e.g. `CoolEnvironment`) for the project
- Click on "Create Project"
- Once the project is created, navigate to the folder where it was created (`~/code/CoolEnvironment`)
- Open file `CoolEnvironment.uproject` with the Unreal Engine Editor (default or select by right click)
- If you are prompted for "Missing Modules" and asked "Would you like to rebuild them now?", choose "Yes"
- Close the Unreal Engine Editor
- Copy AirSim's Unreal Plugins folder into the new project

```bash
cp -r ~/AirSim/Unreal/Plugins ~/code/CoolEnvironment
```

> If you want to update the AirSim plugin automatically, I recommend using a symlink instead of copying the plugin. Check the [install script](https://github.com/pats-drones/pats/blob/simulator/config/install_sim.sh) for an example.

- Open `CoolEnvironment.uproject` with a text editor

```bash
code ~/code/CoolEnvironment/CoolEnvironment.uproject
```

- Replace its content with the following (making sure that `"Name"` under `"Modules"` matches the project)

```json
{
    "FileVersion": 3,
    "EngineAssociation": "4.24",
    "Category": "",
    "Description": "",
    "Modules": [
        {
            "Name": "CoolEnvironment",
            "Type": "Runtime",
            "LoadingPhase": "Default",
            "AdditionalDependencies": [
                "AirSim"
            ]
        }
    ],
    "Plugins": [
        {
            "Name": "AirSim",
            "Enabled": true
        }
    ]
}
```

- Open `CoolEnvironment.uproject` with the Unreal Engine Editor again
- On menu "Select Unreal Engine Verion", confirm the location of your UE4's build (usually `~/code/UnrealEngine`), click "Ok"
- If you are prompted for "Missing Modules" (AirSim in this case) and asked "Would you like to rebuild them now?", choose "Yes"
- When the Editor launches, you'll see a prompt "New plugins are available" on the bottom right of the screen
- Clicking on "Manage plugins" will open a menu showing that AirSim is *installed* and *enabled* (close this menu)
- In the "World Outliner" level editor (top right by default or under the "Window" menu) search bar, type "Player Start"
- Click on the "Player Start" result to open the "Details" tab
- The origin position for AirSim's vehicles is the "Location" under "Transform"
- If a "Player Start" does not exist, add it from the "Modes" tab (on the top left)
- From the "Window" dropdown menu, click on "World Settings"
- In the "GameMode Override" dropdown menu, select "AirSimGameMode"
- Save the project (`Ctrl`+`s`)
- You will be prompted to save the level with a name (e.g. `CoolEnvironmentLvl1`) under `Content`, click "Save"
- Make this level the default start: from menu "Edit" select "Project Settings"
- On the left of the new window, under "Project", select "Maps & Modes"
- On the right, under "Default Maps", select `CoolEnvironmentLvl1` from the dropdown menu for the "Editor Startup Map"
- Also make "AirSimGameMode" the "Default GameMode" and `CoolEnvironmentLvl1` the "Game Default Map", if desired
- Close the "Project Settings" menu, save the project (`Ctrl`+`s`)

### Creating a new level

Adding a level is useful if you want a different environment, for example different plants or a different greenhouse, and so on.

1. Open the Unreal editor
2. Go in to the `Maps` folder
3. Duplicate `GreenhouseEmpty` and give the new map a name
4. Click the `Build` button. Now save the project and a `Mapname_BuiltData` file should appear.

Now you can use `--airsim Mapname` to run the simulator with the new map.

### Updating AirSim

AirSim is constantly being developed. To pull updated source code and re-compile it

```bash
cd ~/AirSim
git remote add upstream https://github.com/microsoft/airsim
git fetch upstream
git checkout master
git merge upstream/master
./build.sh
```

Now you can rebase the `simulator` branch to the `master` branch.

### Betaflight software-in-the-loop

Betaflight has [software-in-the-loop](https://github.com/pats-drones/betaflight/tree/master/src/main/target/SITL) a feature. Instead of Gazebo as a Simulator, we want to use AirSim. AirSim [supports](https://ardupilot.org/dev/docs/sitl-with-airsim.html) ArduCopter as a firmware. I tried compiling the Betaflight SITL with

```bash
run make TARGET=SITL
```

On the Pats fork this gave a seg fault when running. On the current master of Betaflights own repository, this does not happen. With that working I updated the `settings.json` to the following

```json
{
  "SettingsVersion": 1.2,
  "LogMessagesVisible": true,
  "SimMode": "Multirotor",
  "Vehicles": {
    "Copter": {
      "VehicleType": "ArduCopter",
      "UseSerial": false,
      "LocalHostIp": "127.0.0.1",
      "UdpIp": "127.0.0.1",
      "UdpPort": 9003,
      "ControlPort": 9002
    }
  }
}
```

> Initially, the editor will hang after pressing the Play button if the ArduPilot SITL hasn’t been started (this is due to Lock-Step Scheduling).
The above is stated in the docs of ardupilot. Unfortunately, the editor never stops hanging for me.

## Troubleshooting

<details>
<summary>Error Cannot find compatible Vulkan driver (ICD) when running UE4</summary>
<p>

Please install the `mesa-vulkan-drivers` on your system and make sure the correct videocard driver is installed for your device.

**Nvidia**

If you have a nvidia videocard please see [this](https://askubuntu.com/questions/1142530/how-to-install-nividia-graphics-driver-for-gp107m-geforce-gtx-1050-ti-mobile-o) guide for installing the correct driver. A known driver to work is installed with the following command:

```bash
sudo apt install nvidia-driver-430
```

**OpenGL**

If your device does not support Vulkan. please use the `-opengl4` flag . You need to edit `.bash_aliases` and add the flag there.

```bash
/path/to/Simulator/Greenhouse.sh -ResX=640 -ResY=480 -windowed -opengl
```

This will give you the error `Trying to force OpenGL RHI but the project does not have it in TargetedRHIs list`. To fix this see the troubleshooting guide.
</p>
</details>

<details>
<summary>UE4 stuck in initializing at 18%</summary>
<p>

At 18%, Unreal Engine checks for uncompiled shader systems, if so, it will retarget all shaders, in order to prevent future errors, and this process may take a while, so it will look like it's stuck, but it's actually retargeting shaders, just leave it some time. It is possible to confirm this looking at the task manager, where you should see near 100% usage.

</p>
</details>

<details>
<summary>UE4 stuck in initializing at 45%</summary>
<p>

See UE4 stuck in initializing at 18%.

</p>
</details>

<details>
<summary>Unreal Engine freezes after loading</summary>
<p>

UE 4.24 uses Vulkan drivers by default, but they can consume more GPU memory. If you get memory allocation errors, then you can try switching to OpenGL using `-opengl`.

</p>
</details>

<details>
<summary>Fatal error: filesystem: No such file or directory #include filesystem</summary>
<p>

The header `#include <filesystem>` is available in gcc7 as `#include <experimental/filesystem>`. We can solve this problem by compiling with gcc8.<br>
In vscode, press ctrl+shift+p, search for `Cmake: select kit`, select `GCC 8.4`. If that is not available you may need to scan for kits first: `Cmake: Scan for kits`.

</p>
</details>

<details>
<summary>The simulation or the Unreal Engine freezes my laptop</summary>
<p>

Try using the `-opengl` flag when starting UE4. However, it is recommended to use Vulkan, as OpenGL is deprecated.

</p>
</details>

<details>
<summary>Trying to force OpenGL RHI but the project does not have it in TargetedRHIs list</summary>
<p>

Open the following file

```bash
~/code/UnrealEngine/Engine/Config/BaseEngine.ini
```

Uncomment the following line:

```yaml
; OpenGL4 is deprecated, you can comment this back in to add it to your targeted RHI list
+TargetedRHIs=GLSL_430
```
</p>
</details>

<details>
<summary>Error compiling AirSim to binaries</summary>
<p>

When getting the following error, this is a bug in the Unreal Engine code, which has been fixed in the 4.25 release. Make the changes found in [this](https://github.com/EpicGames/UnrealEngine/commit/cb388710a7fbe43eaa82a6d8c43b1632f25f6386) commit to fix the issue.

```cpp
In file included from /home/niek/code/UnrealEngine/Engine/Source/Runtime/Experimental/GeometryCollectionEngine/Private/GeometryCollection/PhysicsAssetSimulation.cpp:14:
[2020.10.05-13.36.34:747][462]UATHelper: Packaging (Linux):   In file included from Runtime/Engine/Classes/Components/SkeletalMeshComponent.h:16:
[2020.10.05-13.36.34:747][462]UATHelper: Packaging (Linux):   /home/niek/code/UnrealEngine/Engine/Source/Runtime/Engine/Public/Animation/AnimCurveTypes.h:164:3: error: declaration shadows a variable in namespace 'Chaos' [-Werror,-Wshadow]
[2020.10.05-13.36.34:747][462]UATHelper: Packaging (Linux):                   X = 0,
[2020.10.05-13.36.34:747][462]UATHelper: Packaging (Linux):                   ^
[2020.10.05-13.36.34:747][462]UATHelper: Packaging (Linux):   /home/niek/code/UnrealEngine/Engine/Source/Runtime/Experimental/Chaos/Public/Chaos/ParticleDirtyFlags.h:12:3: note: previous declaration is here
[2020.10.05-13.36.34:747][462]UATHelper: Packaging (Linux):                   X =                              1 << 0,
[2020.10.05-13.36.34:747][462]UATHelper: Packaging (Linux):                   ^
[2020.10.05-13.36.34:747][462]PackagingResults: Error: declaration shadows a variable in namespace 'Chaos' [-Werror,-Wshadow]
[2020.10.05-13.36.39:544][750]UATHelper: Packaging (Linux):   1 error generated.
```
</p>
</p>
</details>

<details>
<summary>'This project requires plugin, which could not be found.'</summary>
<p>

You need to copy the AirSim plugin to the root folder of the environment.

```bash
cp -r ~/code/If you are prompted for "Missing Modules" and asked "Would you like to rebuild them now?", choose "Yes"AirSim/Unreal/Plugins code/AirSim/Unreal/Environments/Greenhouse/
```

</p>
</details>

<details>
<summary>'Missing Modules' and asked 'Would you like to rebuild them now?'</summary>
<p>

If you are prompted for "Missing Modules" and asked "Would you like to rebuild them now?", choose "Yes"

</p>
</details>

<details>
<summary>Low FPS</summary>
<p>

1. Check if the lighting in the scene needs to be rebuilt
2. Check the FPS of the simulator itself, the higher the FPS in-game the higher the FPS in the Pats software.

</p>
</details>

<details>
<summary>Waiting for connection - XXXXX</summary>
<p>

Make sure the simulation is running. Use the play button in the Unreal Editor and make sure you see a black screen.

</p>
</details>

<details>
<summary>Error: using airsim mode but AirSim is not compiled. Add -DWITH_AIRSIM=TRUE flag when cmaking.</summary>
<p>

Run `cmake .. -DWITH_AIRSIM=TRUE` in the folder `config/pc/build-vscode`.

</p>
</details>

<details>
<summary>error:CDO Constructor(PIPcamera):failed to find Material'AirSim/HUDAssets/CameraDistortion.CameraDistortion'</summary>
<p>

Update your UE version to 4.25, and the error will disappear. See [updating the simulator](#updating-the-simulator).
</p>
</details>

<details>
<summary>The simulator fails to launch</summary>
<p>

1.

```bash
SearchForPackageOnDisk took   0.022s, but failed to resolve Blocks.
terminating with uncaught exception of type std::__1::ios_base::failure: No such file or directory: unspecified iostream_category error
```

If you get the above error, AirSim is not able to find a `settings.json` file. This file should be located `~/Documents/AirSim/settings.json`.

</p>
</details>

<details>
<summary>No spin up/take off/lift off detected</summary>
<p>

On line 107-110 in `dronetracker.cpp`, change the values `0.3` and `0.35` to for example `1.0` and `1.4`.

```cpp
if(spinup_detected<3 && takeoff_duration > dparams.full_bat_and_throttle_spinup_duration + 0.3f) { // hmm spinup detection really does not work with improper lighting conditions. Have set the time really high. (should be ~0.3-0.4s)
    _take_off_detection_failed = true;
    std::cout << "No spin up detected in time!" << std::endl;
} else if (takeoff_duration > dparams.full_bat_and_throttle_spinup_duration + 0.35f) {
```

</p>
</details>

## Examples

### moveByRC

Example of moveByRC

```cpp
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <iostream>
#include <chrono>

int main()
{
    using namespace msr::airlib;
    msr::airlib::MultirotorRpcLibClient client;
    RCData rc_data;

    try {
        client.confirmConnection();
        rc_data.is_initialized = true;
        rc_data.is_valid = true;

        rc_data.throttle = 1;
        client.moveByRC(rc_data);

        std::chrono::seconds timespan(3);
        std::this_thread::sleep_for(timespan);

        rc_data.throttle = 0.5;
        client.moveByRC(rc_data);
    }
    catch (rpc::rpc_error&  e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
    }

    return 0;
}
```
