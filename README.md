# UVDAR core (ROS 2) — fork notes

![](.fig/thumbnail.jpg)

Fork of [ctu-mrs/uvdar_core](https://github.com/ctu-mrs/uvdar_core) (`ros2_devel` branch),
extended for a **multi-camera mvBlueFOX rig** that outputs calibrated bearings in the body
frame. This README documents *this fork*: how to build it, which environment variables it
reads, which config files to edit for your own rig, and how to launch and check it.

Upstream docs for the system in general: <https://github.com/ctu-mrs/uvdar_core>.

**What this fork adds on top of upstream `ros2_devel`:**

| Addition | File |
|---|---|
| Three-camera rig launch (left / right / back) with static TF mounts and the bearing stage included | [launch/three_bluefox.launch.py](launch/three_bluefox.launch.py) |
| Vehicle-agnostic three-camera config (`$UAV_NAME` everywhere, no hardcoded `uav4`) | [config/three_bluefox_bearing.yaml](config/three_bluefox_bearing.yaml) |
| Camera calibs referenced by real serial instead of dangling absolute-path symlinks | `config/camera/bluefox_ocam_calib/bf_uv_*.yaml` |
| Beacon target model used on the UAV | `config/models/quadrotor_foursided_BEACON_px4_motor_aligned.txt` |
| Camera bring-up + serial/exposure/bashrc helper script | [install/setupBluefox.sh](install/setupBluefox.sh) |
| Static TF example for onboard sensors | [launch/x500_sensor_TFs.launch.py](launch/x500_sensor_TFs.launch.py) |

---

## 1. What the package is

One processing pipeline, shipped as separate executables that you compose by topic:

```
camera (bluefox2)                                        downstream
   image_raw ──► detector_node ──► tracker_node ──► bearing_node ──► uvdar_fusion / RViz / ...
                   points_seen        blinkers         observations
                                        │
                                        └──► pose_estimator_node ──► filter_node
```

* **bearing endpoint** (`detector` → `tracker` → `bearing`) — what this rig actually runs.
  One `detector_node`, one `tracker_node`, one `bearing_node` per vehicle, each iterating over
  the camera list in the config.
* **pose endpoint** (`pose_estimator` → `filter`) — full relative 6-DoF pose from the
  bearings + the target's LED model. Implemented but not launched by this fork; the bearing
  node's single-topic output is what the multi-camera composition consumes.

Nodes: `detector_node`, `tracker_node`, `bearing_node`, `pose_estimator_node`, `filter_node`,
`calibrator_node`, `led_manager_node`.

Key interfaces:

| Topic | Type | Notes |
|---|---|---|
| `uvdar/bearing/observations` | `uvdar_core/BearingObservationArrayStamped` | published in `bearing.output_frame`, **one topic for all cameras** — one message per camera per callback, each carrying its own `origin`; consumers must merge using `origin` |
| `uvdar/pose_estimator/measured_poses` | `uvdar_core/PoseWithCovarianceArrayStamped` | pose endpoint output |
| `uvdar/filter/filtered_poses` | same | filtered, validated tracks |

---

## 2. System requirements

**Hardware**

* One or more **mvBlueFOX MLC200wG** cameras with UV bandpass filter (Midopt BP365-R6) and
  fisheye lens (Sunnex DSL215, ~180° HFOV), one per rig slot (left / right / back).
* Blinking UV markers (395 nm) on the target's arm tips — driven by
  [`mrs_hw_uvdar`](https://github.com/ctu-mrs/mrs_hw_uvdar) LED boards.
* A GPU with a `/dev/dri/renderD*` node for the `gpu` detector backend
  (`cpu` backend works without one — and still JIT-compiles its kernel with `cc` at startup).

**Software** — ROS 2 **Jazzy** (what this fork is built against), plus:

* [`bluefox2`](https://github.com/ctu-mrs/bluefox2) — camera driver, **built from source** in
  the same workspace. On x86_64 the mvIMPACT Acquire SDK it links against is vendored in that
  repo (`install/x86_64/mvIMPACT_acquire-x86_64-2.48.0`) and installed into the package's
  `lib/`, so no separate SDK install is needed; `bluefox2/install/install.sh` is for ARM or for
  updating that vendored copy.
* `ros-jazzy-mrs-msgs`, `ros-jazzy-mrs-modules-msgs`, `ros-jazzy-mrs-serial` — MRS messages
  and the LED serial bridge. `rosdep` resolves these only if the CTU-MRS source list is
  registered (`rosdep resolve mrs_msgs` must work). Either run the
  [MRS installation script](https://github.com/ctu-mrs/mrs_uav_system), or add
  `/etc/ros/rosdep/sources.list.d/ctu-mrs-stable.list` pointing at
  `https://ctu-mrs.github.io/ppa2-stable/{generated_mrs_amd64,generated_thirdparty_amd64,handcrafted_amd64,handcrafted_common}.yaml`
  and `rosdep update`.
* `ros-jazzy-cv-bridge`, `tf2`, `tf2-ros`, `image-view` (camera viewer), `image-proc`
  (optional `rectify:=true`).
* Build deps resolved by CMake: `cmake` (≥ 3.14), a C++20 compiler, `libgl1-mesa-dev`
  (`EGL`, `gbm`, `GLESv2`), `git` — CMake FetchContent pulls
  [CvPlot](https://github.com/Profactor/cv-plot) header-only at configure time. `package.xml`
  does **not** declare OpenCV or Eigen, so `rosdep` will not install them for this package
  (they usually arrive with `cv_bridge`): add `libopencv-dev libeigen3-dev libyaml-cpp-dev`
  explicitly. `glslang-tools` is declared as an `exec_depend` but nothing in the build or at
  runtime calls it — the `.comp` shaders are embedded with `ld -b binary`, not compiled.

---

## 3. Build

The fork is designed to live in a workspace next to `bluefox2`, both symlinked into `src/`
so edits are picked up by `--symlink-install` builds:

```bash
mkdir -p ~/ws_uvdar/src
cd ~/ws_uvdar/src
ln -s ~/git/uvdar_core .
ln -s ~/git/bluefox2 .

cd ~/ws_uvdar
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select bluefox2 uvdar_core --symlink-install
source install/setup.bash
```

Notes:

* **Use `--symlink-install`** — otherwise every config edit you make under `src/uvdar_core`
  needs a rebuild before it reaches `install/…/share/uvdar_core/config/`, which is the copy
  launch actually reads.
* `single_bluefox.launch.py` prepends `/opt/mvIMPACT_acquire_libusb` to `LD_LIBRARY_PATH`
  for the camera container. That path only exists on machines where the mvIMPACT driver was
  installed by `bluefox2/install/install.sh`. With a workspace-built `bluefox2` the driver
  libs resolve through the workspace's own `LD_LIBRARY_PATH` and the extra path is harmless.
* `mrs_serial` must be available before `led_manager.launch.py` will run (it includes
  `mrs_serial/launch/baca_protocol.launch.py`).

---

## 4. `~/.bashrc` — everything you need to add

Every stage reads its **namespace** and several names from the *process environment*, not
from ROS parameters. `helpers/yaml.hpp` expands `$VAR` and `${VAR}` inside every YAML value
and **refuses to load the file**, naming the variable, if it is unset — so a missing export
fails loudly at startup rather than silently producing a frame named `/fcu`.

```bash
# ---- uvdar_core ----
export UAV_NAME=uav13          # vehicle namespace; used in TF frames by $UAV_NAME/fcu etc.

# Camera serials — print them with:  ros2 run bluefox2 bluefox2_list_cameras
export BLUEFOX_LEFT_ID=25001879
export BLUEFOX_RIGHT_ID=25001954
export BLUEFOX_BACK_ID=25002xxx

# Manual exposure per camera, microseconds. aec is forced to false by the
# two/three_bluefox launch files, so these are the real exposure settings.
export EXPOSE_US_LEFT=1000
export EXPOSE_US_RIGHT=1000
export EXPOSE_US_BACK=1000

# Optional
export USE_SIM_TIME=false      # read by every launch file's use_sim_time argument
```

`two_bluefox.launch.py` and `three_bluefox.launch.py` fail at launch if the
`BLUEFOX_*_ID` / `EXPOSE_US_*` for a slot they start is unset — `EnvironmentVariable` has no
default there. `install/setupBluefox.sh` detects the serials one camera at a time and writes
these lines into `~/.bashrc` for you (it also fixes the calibration symlinks; see §7).

If `UAV_NAME` is not set at all, `EnvironmentVariable('UAV_NAME')` aborts the camera launch
with *"environment variable 'UAV_NAME' does not exist"* before any node starts.

**USB permissions** (once, not per shell):

```bash
# /etc/udev/rules.d/99-bluefox.rules
SUBSYSTEM=="usb", ATTR{idVendor}=="164c", ATTR{idProduct}=="0103", MODE="0660", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="164c", ATTR{idProduct}=="0101", MODE="0660", GROUP="plugdev"
```
then `sudo udevadm control --reload && sudo udevadm trigger`, and make sure you are in
`plugdev` (`groups | grep plugdev`).

---

## 5. Which config file is which

All stage launch files take `config_file:=` (a single YAML shared by all three stages of the
endpoint) and `namespace:=` (default `$UAV_NAME`). Relative paths inside a config —
`calib_file`, `model_file`, `sequence_file`, `mask_file` — resolve **against that config
file's directory**, which after install is
`install/uvdar_core/share/uvdar_core/config/`. Keep new files inside `config/` so those
relative references keep working, and rebuild (or rely on `--symlink-install`) after adding
one.

| File | Used by | What it describes |
|---|---|---|
| [three_bluefox_bearing.yaml](config/three_bluefox_bearing.yaml) | `three_bluefox.launch.py` (explicitly) | detector → tracker → bearing for **left/right/back**, vehicle-agnostic |
| [default_bearing.yaml](config/default_bearing.yaml) | `bearing.launch.py` (default) | detector → tracker → bearing for **two cameras**; ⚠ contains hardcoded `uav4` and missing calib files, see §8 |
| [default_bluefox.yaml](config/default_bluefox.yaml) | `detector/tracker/pose_estimator.launch.py` (default) | full-stack config for the two-camera rig incl. pose estimation + filtering |
| [default.yaml](config/default.yaml) | `filter.launch.py` (default) | upstream single-camera default, every key documented inline — the best reference for what each parameter means |
| [calib_default.yaml](config/calib_default.yaml) | `calibrator.launch.py` | camera calibration run |
| `config/selected.txt` | `led_manager.launch.py`, `tracking.sequence_file` | symlink → the blink-sequence library in use |

**Copy, don't edit in place**: make `config/myrig_bearing.yaml` and pass
`config_file:=$(ros2 pkg prefix uvdar_core)/share/uvdar_core/config/myrig_bearing.yaml`
(or the absolute installed path).

---

## 6. Editing the config for your rig — the checklist

### 6.1 Camera topics (`detector.inputs[].input_topic`, `tracking.inputs[].input_image_topic`)

The topic a camera publishes is **not a free choice**. It is
`<uav_name>/<camera_name>/bluefox/image_raw`: the node name is hardcoded as `bluefox` in
`single_bluefox.launch.py`, and `camera_ns` is `<uav_name>/<camera_name>`. So for
`camera_name: left` it is `left/bluefox/image_raw` written *relative to the
`<uav_name>` namespace*. Rename the node or the camera name and the detector reads nothing —
there is no error, just silence.

### 6.2 Frame names that must be the same string in three places

`camera_frame` is a TF frame name, not a topic, so the node namespace does **not** prefix
it. For slot `<S>`:

1. `single_bluefox.launch.py:134-137` builds the driver's `frame_id` as
   `$UAV_NAME/bluefox_<S>`;
2. the rig launch's `static_transform_publisher` publishes
   `--frame-id $UAV_NAME/fcu --child-frame-id $UAV_NAME/bluefox_<S>`;
3. your config says `camera_frame: $UAV_NAME/bluefox_<S>`.

All three must be one string. If the mount for a frame is missing, the bearing node
**does not fail at startup** — it drops that camera's observations with a throttled warning
naming both frames, and RViz shows nothing. Same class of failure for `output_frame`: if it
names a frame that does not exist for your vehicle, every observation is dropped.

### 6.3 Stage topics must chain exactly

`detector.inputs[S].output_topic` == `tracking.inputs[S].input_topic`, and
`tracking.inputs[S].output_topic` == `bearing.inputs[S].input_topic`. Nothing compares these;
a mismatch starts all three processes, prints all their banners, and stays silent forever.

### 6.4 Calibration files

`bearing.inputs[].calib_file` (and `pose_estimation.inputs[].calib_file`) point at an OCamCalib
YAML per camera, e.g. `camera/bluefox_ocam_calib/bf_uv_25001879.yaml`. This fork names the
**real serial-numbered files directly** because the upstream `bf_left.yaml` /
`bf_right.yaml` symlinks were committed with absolute `/home/mrs/…` targets and are dangling
everywhere else. A *missing* calib file stops the bearing node at startup with a lens-model
error — that is deliberate: pointing a camera at the wrong calibration produces plausible,
wrong bearings that look like a bad mount. Verify the mapping against
`ros2 run bluefox2 bluefox2_list_cameras`; left ↔ `25001879`, right ↔ `25001954`.

### 6.5 Blink sequences

`tracking.sequence_file` (default `./selected.txt`, a symlink into `config/sequences/`)
replaces the inline `tracking.sequences` list when set. All sequences must be non-empty and
**equal length**, and this file must be the same library the LED boards on the targets run —
otherwise nothing decodes. Formats accepted inline: `"0011101101"`, `0,0,1,1,...`, a list of
bits, or `len:0xHEX`. `manchester_code: true` doubles the length.

### 6.6 Other values you are likely to touch

| Key | Why |
|---|---|
| `detector.inputs[].backend` | `gpu` (needs a render node) or `cpu` |
| `detector.inputs[].threshold` / `threshold_diff` | FIMD brightness + contrast gates; raise outdoors, lower for dim/far markers |
| `detector.latest_frame_only` | `true` bounds latency by keeping only the newest pending frame per input (present in `default.yaml`, not yet in the bluefox configs) |
| `tracking.implementation` | `ami` (classic) or `generalized` (covariance-aware) — defaults for the other tuning keys switch with this value |
| `tracking.allowed_BER_per_seq` | bit errors tolerated per sequence; raise from `0` if sequences are long / partial occlusions |
| `pose_estimation.model_file` | the **target's** LED geometry, `X Y Z Type Pitch Yaw signal_id`; must match the physical target |
| `pose_estimation.signal_ids` | which ids the estimator accepts |
| `filtering.output_frame` | world-fixed frame the filtered poses live in |
| `detector.gui` / `tracking.gui`, `publish_visualization` | debug windows / annotated images |
| `detector.debug`, `pose_estimation.debug` | extra diagnostic logging, cheap to leave on while bringing up a rig |

---

## 7. First run on a new vehicle / new cameras

1. **Serials + exposure into `~/.bashrc`.** Either edit them by hand (§4) or:
   ```bash
   bash install/setupBluefox.sh
   ```
   It asks you to connect one camera at a time, reads its serial, points
   `bf_left.yaml`/`bf_right.yaml` at `bf_uv_<serial>.yaml` (recreating the links §8 says were
   deleted — useful if you would rather keep `default_bearing.yaml`'s names), writes the four
   `BLUEFOX_*` / `EXPOSE_US_*` exports into `~/.bashrc`, launches the two-camera rig and checks
   `ros2 topic hz`. Then `source ~/.bashrc`. **Edit the variables at the top of the script
   first** — `WORKSPACE` is hardcoded to `/home/$USER/ws_tim`, and `LAUNCH_FILE` is
   `two_bluefox.launch.py` (it does not know about the third camera).
2. **Cameras only:**
   ```bash
   ros2 launch uvdar_core two_bluefox.launch.py        # uses $UAV_NAME
   ros2 topic hz /$UAV_NAME/left/bluefox/image_raw
   ```
3. **Check the TF tree** before blaming the estimator:
   ```bash
   ros2 run tf2_tools view_frames.py
   # expect <uav>/fcu -> <uav>/bluefox_{left,right,back}
   ```
4. **The full bearing endpoint** — cameras, three mounts, detector, tracker and bearing in
   one command:
   ```bash
   ros2 launch uvdar_core three_bluefox.launch.py
   ros2 topic hz /$UAV_NAME/uvdar/bearing/observations
   ```
5. **Stages separately**, when debugging one link of the chain (all three share one
   `config_file`):
   ```bash
   # all three stages, with the option to suppress two of them
   ros2 launch uvdar_core bearing.launch.py config_file:=<...> namespace:=$UAV_NAME \
       launch_detector:=false launch_tracker:=false
   # or one stage on its own
   ros2 launch uvdar_core detector.launch.py config_file:=<...> namespace:=$UAV_NAME
   ```
6. **Visualization** — RViz with the `uvdar_core/TrackerOutput` and
   `BearingObservationArrayStamped` displays/markers, or the annotated images on
   `uvdar/<side>/tracker/visualization` (enable with `publish_visualization: true`).

New camera without a calibration? Run the calibrator and put the output where the config
expects it:

```bash
ros2 launch uvdar_core calibrator.launch.py
ros2 topic echo /$UAV_NAME/calibrator/status     # progress; result dashboard shown for a few seconds
# output file + pattern settings: config/calib_default.yaml (image_topic, output_calibration_file,
# calibration_model: ocamcalib, pattern_rows/columns/spacing, required_pattern_frames)
```

Target-side LED boards (blinking sequence, frequency, on/off) are driven by
`ros2 launch uvdar_core led_manager.launch.py led_serial_port:=/dev/MRS_MODULE1` — services
under `/$UAV_NAME/led_manager/`: `set_active`, `set_frequency`, `load_sequences`,
`select_sequences`, `quick_start`, `set_mode`, `set_message`.

---

## 8. Things that are broken or fiddly on purpose

* **`config/default_bearing.yaml` is currently unusable as-is.** It reads
  `camera/bluefox_ocam_calib/bf_left.yaml` / `bf_right.yaml`, which were deleted in
  `8290d68` (they were dangling absolute-path symlinks) — the bearing node exits at startup.
  It also pins `output_frame: uav4/fcu` and an absolute `/uav4/...` `output_topic`, so for any
  other vehicle every observation is dropped. Either repoint its `calib_file`s at the
  `bf_uv_<serial>.yaml` files and its frame/topic at `$UAV_NAME`, or use
  `three_bluefox_bearing.yaml` (which already does neither of those things wrong).
  `config/default_bluefox.yaml`'s `pose_estimation.inputs` references the same missing
  `bf_left/bf_right` files.
* **The `back` camera has no calibration file.** `three_bluefox_bearing.yaml` points at
  `camera/bluefox_ocam_calib/bf_back.yaml`, which does not exist — deliberately, so the node
  stops with a lens-model error instead of publishing plausible-but-wrong bearings. Calibrate
  it (§7) or remove that input.
* **Bearings from several cameras share one topic and are not merged.** `bearing_node` reads
  `bearing.output_topic` once and publishes every input through that single publisher, per
  camera callback: a blinker visible in two cameras arrives as two unordered messages, each
  with its own `origin`. Merging is the consumer's job (that is what `origin` is for).
* **A per-input `output_topic:` under `bearing:` is never read** — the ones in
  `default_bearing.yaml` are inert. Listing one only advertises a topic that cannot exist.
* **GPU detection needs a render node**; if `/dev/dri/renderD*` is missing or unreadable the
  detector backend fails to initialise. Check `groups` for `render`/`video` (or an ACL on
  the node), or set `backend: cpu`.
* **`/tmp/uvdar_core_fimd_cache`** holds JIT-compiled CPU kernels keyed by resolution and
  thresholds. After changing image size or thresholds, expect a compile pause on the first
  frame; delete the directory if a stale cache misbehaves.
* **Zenoh / DDS.** This machine runs `RMW_IMPLEMENTATION=rmw_zenoh_cpp` with a shared router
  (`ZENOH_ROUTER_CONFIG_URI`). Camera + detector + tracker + bearing on the same host work
  best in one router (or with intra-process comms via `standalone:=false` and a shared
  container); cross-host bearing topics need the router config to actually connect the hosts.
* **`ros2 topic hz` on 60 Hz `CompressedImage`/`Image` from Python** can report a lower rate
  than the camera delivers; `ros2 topic info -v` (matching `qos profile`) is the reliable
  check for a silent link.

---

## 9. Debugging matrix

| Symptom | Most likely cause | Check |
|---|---|---|
| Node dies: *"Environment variable 'X' referenced by YAML configuration is not set."* | `~/.bashrc` export missing | §4 |
| Launch dies: *"environment variable 'UAV_NAME' does not exist"* | `UAV_NAME` unset, camera launch has no default | `echo $UAV_NAME` |
| `image_raw` silent | wrong `BLUEFOX_*_ID`, udev permissions, camera not enumerated | `ros2 run bluefox2 bluefox2_list_cameras` |
| Detector publishes nothing | `input_topic` doesn't match `<ns>/<side>/bluefox/image_raw` | `ros2 topic info <t> -v` |
| Tracker publishes, bearing silent (throttled TF warning) | `camera_frame` ≠ child frame of the published mount | `view_frames.py`, `ros2 run tf2_ros tf2_echo <uav>/fcu <uav>/bluefox_left` |
| Bearings all dropped, frame named in warning | `bearing.output_frame` belongs to a different vehicle | config `output_frame` vs your `$UAV_NAME` |
| Bearing node won't start, lens-model error | missing/renamed `calib_file` | `ls install/uvdar_core/share/uvdar_core/config/camera/bluefox_ocam_calib/` |
| Nothing ever identified (`id: -1`) | sequence library ≠ what the target broadcasts | `tracking.sequence_file` |
| Targets jitter/alternate between cameras downstream | multi-camera bearings are unmerged on one topic | merge using `origin` in the consumer |

---

## 10. Syncing with upstream

```bash
git remote -v        # origin = Red-mrs/uvdar_core-1, upstream = ctu-mrs/uvdar_core
git fetch upstream
git merge upstream/ros2_devel     # this fork's tracking branch is ros2_devel
```

Note that `three_bluefox.launch.py` and `three_bluefox_bearing.yaml` exist only on
`ros2_bluefox_wip` — `ros2_devel` still has the two-camera rig and no `three_bluefox` config.
Merge in one direction (`upstream/ros2_devel` → work branch), and keep the work branch merged
back before switching vehicles, otherwise a launch command that worked yesterday is not found.

Upstream touches `config/default_bearing.yaml` and the camera calibration symlinks from time
to time — after a merge, re-apply this fork's two conventions before flying: real calib files
instead of symlinks, and `$UAV_NAME` instead of a literal vehicle name.

## License

BSD-3-Clause, see [LICENSE](LICENSE). Original work by CTU MRS; this fork maintained by
Radomír Nový.
