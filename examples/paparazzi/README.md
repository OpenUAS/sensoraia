# Paparazzi Simulation & Integration Manual

Fly a complete Sensoraia mission from **start to landing** on your Linux PC
using Paparazzi Center—with no MORA board, no physical camera, and no Python
simulation helpers required. Simulated propellers never snap, batteries never
freeze, and you run the exact same native C/C++ USHER and sensor code that
deploys to MORA. When you connect a MORA board over an FTDI USB-UART cable
later, the very same Paparazzi Center sessions talk straight to real USHER on
MORA.

---

## 1. Build Native Sensoraia Applications

Make sure Paparazzi is already installed and built on your Linux PC (typically
at `~/paparazzi`), and install the [Sensoraia PC prerequisites](../../docs/README.md#2-install-pc-dependencies).
From your Sensoraia checkout:

```sh
cd ~/develop/sensoraia
git submodule update --init --recursive
make native
```

`make native` compiles `usher` and every PC sensor application (`bin/spottercam`,
`bin/aicam`, `bin/lwircam`, `bin/chdkbridge`, `bin/earcam`, `bin/loomcam`,
`bin/diplacam`, `bin/pangea`, `bin/camera_lens`, and `bin/soda`).

---

## 2. Link Sensoraia Into Paparazzi (`./kickstart.sh` Or Manual Symlinks)

Paparazzi Center discovers aircraft configurations and control panels under
`$PAPARAZZI_HOME/conf/userconf/`, and resolves flight plans, airframes, radios,
and telemetry definitions under `$PAPARAZZI_HOME/conf/`.

### Option A: Automatic Setup With `./kickstart.sh` *(Recommended)*

From the Sensoraia root:

```sh
./kickstart.sh
```

If your Paparazzi checkout is somewhere other than `~/paparazzi`, set
`PAPARAZZI_HOME=/path/to/paparazzi ./kickstart.sh`.

- `./kickstart.sh` links the Sensoraia checkout (`sw/ext/sensoraia`), flight
  plans, and self-contained demo airframes/radios/telemetry into
  `$PAPARAZZI_HOME/`, copies `sensoraia_demo_conf.xml` and the unified
  `sensoraia_demo_control_panel.xml` into `conf/userconf/SENSORAIA/` (if not
  already present) so Paparazzi Center lists them, and points `conf/conf.xml`
  and `conf/control_panel.xml` to them (saving backups of any previous links
  under `~/.local/state/sensoraia/kickstart/`).
- Want to install the links **without** changing your active `conf.xml` or
  `control_panel.xml`? Run `./kickstart.sh --no-activate` and select the
  Sensoraia XML files from Paparazzi Center's dropdown menus manually. Pass
  `./kickstart.sh --refresh` if you want to overwrite your copied `userconf`
  XML files with the latest shipped templates (preserving `.previous.*` backups).

### Manual Symlink Setup

If you prefer to create every link yourself instead of running `./kickstart.sh`,
run these commands:

```sh
export PAPARAZZI_HOME="$HOME/paparazzi"
export SENSORAIA_ROOT="$HOME/develop/sensoraia"

mkdir -p "$PAPARAZZI_HOME/conf/userconf/SENSORAIA" "$PAPARAZZI_HOME/sw/ext"

# 1. Symlink Sensoraia into sw/ext/sensoraia plus flight plans, airframes, radio, and telemetry
ln -s "$SENSORAIA_ROOT" \
      "$PAPARAZZI_HOME/sw/ext/sensoraia"
ln -s "$SENSORAIA_ROOT/examples/paparazzi/conf/flight_plans/SENSORAIA" \
      "$PAPARAZZI_HOME/conf/flight_plans/SENSORAIA"
ln -s "$SENSORAIA_ROOT/examples/paparazzi/conf/airframes/SENSORAIA" \
      "$PAPARAZZI_HOME/conf/airframes/SENSORAIA"
ln -s "$SENSORAIA_ROOT/examples/paparazzi/conf/radios/SENSORAIA" \
      "$PAPARAZZI_HOME/conf/radios/SENSORAIA"
ln -s "$SENSORAIA_ROOT/examples/paparazzi/conf/telemetry/SENSORAIA" \
      "$PAPARAZZI_HOME/conf/telemetry/SENSORAIA"

# 2. Copy the demo aircraft list and unified control panel so Paparazzi Center discovers them
cp "$SENSORAIA_ROOT/examples/paparazzi/conf/userconf/SENSORAIA/sensoraia_demo_conf.xml" \
   "$PAPARAZZI_HOME/conf/userconf/SENSORAIA/sensoraia_demo_conf.xml"
cp "$SENSORAIA_ROOT/examples/paparazzi/conf/userconf/SENSORAIA/sensoraia_demo_control_panel.xml" \
   "$PAPARAZZI_HOME/conf/userconf/SENSORAIA/sensoraia_demo_control_panel.xml"

# 3. Optional: point active conf.xml and control_panel.xml at the Sensoraia files
ln -sfn userconf/SENSORAIA/sensoraia_demo_conf.xml "$PAPARAZZI_HOME/conf/conf.xml"
ln -sfn userconf/SENSORAIA/sensoraia_demo_control_panel.xml "$PAPARAZZI_HOME/conf/control_panel.xml"
```

---

## 3. Choose Your Camera In Paparazzi Center

There is **one** unified control panel
(`userconf/SENSORAIA/sensoraia_demo_control_panel.xml`) and **one** demo
aircraft configuration (`userconf/SENSORAIA/sensoraia_demo_conf.xml`). Pick the
row below that matches the camera you want to simulate:

| Camera You Want To Use | Select **A/C** (Aircraft) | Select **Session** | Flight Plan (`flight_plans/SENSORAIA/`) |
| --- | --- | --- | --- |
| **Spottercam** (IMX500 + vehicle detection) | `Easystar_3_SPOTTERCAM` | **Sensoraia Spottercam NPS** | `sensoraia_spotter_cam_demo.xml` |
| **AI Camera** (plain visible photos) | `Easystar_3_AICAM` | **Sensoraia AIcam NPS** | `sensoraia_ai_cam_demo.xml` |
| **LWIRcam** (Tiny1-C / TDK thermal) | `Easystar_3_LWIRCAM` | **Sensoraia LWIRcam NPS** | `sensoraia_lwir_cam_demo.xml` |
| **CHDKbridge** (Canon USB zoom + capture) | `Easystar_3_CHDKCAM` | **Sensoraia CHDKcam NPS** | `sensoraia_chdk_cam_demo.xml` |
| **EARcam** (acoustic beacon search) | `Easystar_3_EARCAM` | **Sensoraia EARcam NPS** | `sensoraia_ear_cam_demo.xml` |
| **LOOMcam** (visual outbound/return) | `Easystar_3_LOOMCAM` | **Sensoraia LOOMcam NPS** | `sensoraia_loom_cam_demo.xml` |
| **MIMIC + PANGEA** (cached map to orthomap) | `Easystar_3_MIMIC` | **Sensoraia MIMIC PANGEA NPS** | `sensoraia_mimic_pangea_demo.xml` |
| **Diplacam** (rotorcraft 8-point planner) | `Bebop_DIPLACAM` | **Sensoraia Diplacam NPS** | `sensoraia_dipla_cam_demo.xml` |

Hardware sessions live in the same panel:
- **Sensoraia MORA desk NPS** / **Sensoraia Diplacam MORA desk NPS** — run
  `Simulator`, `Data Link`, `Server`, and `PprzGCS` over an FTDI USB-UART cable
  (`/dev/ttyUSB0`) to a real MORA board (no PC `USHER`). See the
  [FTDI desk-test guide](../../docs/desk-test.md).
- **Sensoraia MORA realflight** / **Sensoraia Diplacam MORA realflight** /
  **Sensoraia Diplacam MORA Real Cyberzoo Flight** — run ground-station tools
  (`Data Link`, `Server`, `PprzGCS`, plus telemetry/MoCap tools) for real
  aircraft flights.

---

## 4. Fly From Start To Landing

1. **Start Paparazzi Center:**

   ```sh
   cd ~/paparazzi
   ./paparazzi
   ```

2. **Select Configuration & Build `nps`:**
   - Check that **Config** is `userconf/SENSORAIA/sensoraia_demo_conf.xml` and
     **Control panel** is `userconf/SENSORAIA/sensoraia_demo_control_panel.xml`
     (restart Center if it was already open during setup).
   - Select your **A/C** from the table above (for example
     `Easystar_3_SPOTTERCAM`), select target **`nps`**, and click **Build** (or
     build from a terminal with
     `make -C ~/paparazzi -f Makefile.ac AIRCRAFT=Easystar_3_SPOTTERCAM nps.compile`).

3. **Execute the Matching Session:**
   - Switch to the **Session** tab, select the matching session (for example
     **Sensoraia Spottercam NPS**), and click **Execute**.
   - All five native programs start directly: `Simulator` (`pprzsim-launch`),
     `Data Link`, `Server`, `PprzGCS`, and `USHER` (`sw/ext/sensoraia/usher`).
     Every `USHER` flag is visible and editable right in the Session tab's
     command bar. Do not launch a second `USHER` in another terminal.

4. **Take Off, Fly the Mission, and Land in `PprzGCS`:**
   - **Fixed-wing demos (`Easystar_*`):**
     1. Wait a few seconds until the aircraft appears on the map and `Wait GPS`
        switches to **`Holding point`**.
     2. Click **`Takeoff`** in the left GCS strip.
     3. The aircraft launches, climbs to mission altitude, and automatically
        enters **`Start demo`** (survey legs for Spottercam/AIcam/LWIR/CHDK/MIMIC,
        flower search for EARcam, or outbound-then-return for LOOMcam). Watch
        saved JPEGs appear in `~/Pictures/sensoraia/` and target waypoints
        (`T1..T5` or `TARGET`) update on the map where applicable.
     4. When the mission block finishes, the flight plan automatically enters
        **`Land`**, computes the baseleg turn, glides down **`Final`** (`AF`
        $\rightarrow$ `TD`), and flares to a full stop on the runway. (You can
        also click **`Standby`** or **`Land`** at any time.)
   - **Rotorcraft Diplacam demo (`Bebop_DIPLACAM`):**
     1. Wait for **`Ground standby`**, make sure **`NAV`** mode is active, and
        click **`Takeoff`** to hover at 2 m in **`Standby`**.
     2. Click **`Dipla`**: the quadcopter hovers at `CENTRE`, anchors the field,
        receives eight native waypoints (`P1..P8`), and flies **`Follow eight`**.
     3. Click **`Return`** to fly back to `HOME`, then click **`Land Here`** and
        wait until it descends into **`Ground standby`** with motors off.

5. **Stop Cleanly:** Click **Stop** (or **Remove All**) in Paparazzi Center to
   shut down all session processes before starting another flight. (Remember:
   closing USHER alone while airborne only turns off the camera operator—the
   autopilot keeps flying the airplane until you command **Standby** or
   **Land**!)

The fixed-wing demos use an illustrative Valkenburg origin and 100 m nominal
altitude for simulation. Always design and verify site-specific routes, turn
radii, altitudes, and safety limits before flying real hardware.

---

## 5. MIMIC Terrain Cache Check (For `Easystar_3_MIMIC` & `Easystar_3_LOOMCAM`)

For **Sensoraia LOOMcam NPS**, the nominal AIcam lens/shutter profile
(`--mock-lens resources/camera_lens_aicam.json`) is passed directly on the
`USHER` line in the Session tab. Edit that `--mock-lens` path in the Session
command bar to try another lens JSON, or remove `--mock-lens ...` for a clean
pinhole baseline. See
[LOOM camera-profile setup](../../modules/loom_cam/docs/loom-cam.md#select-the-simulated-camera-profile)
for calibration and timing details.

Both **Sensoraia LOOMcam NPS** and **Sensoraia MIMIC PANGEA NPS** render
camera-view photographs from zoom-18 satellite tiles cached by `pprzgcs` in
`~/.local/share/paparazzi/pprzgcs/map/GOOGLE/18` (and **MIMIC PANGEA NPS** also
feeds saved photos into `bin/pangea` at `~/Pictures/sensoraia/pangea-mimic`).
Before flying offline, pan over your survey area at zoom 18 in `PprzGCS` once
while online and confirm zoom-18 JPEGs exist:

```sh
find ~/.local/share/paparazzi/pprzgcs/map/GOOGLE/18 -name '*.jpeg' | head
```

See the [MIMIC guide](../../docs/mimic.md) and
[PANGEA guide](../../modules/pangea/docs/pangea.md).

---

## 6. Using Your Own Paparazzi Airframe

If you want to fly **your own** working Paparazzi aircraft instead of the
supplied `Easystar_*` or `Bebop_DIPLACAM` entries:

1. Add `<module name="digital_cam_uart">` to your airframe XML (see
   [Add The Paparazzi Module To Your Own Aircraft](../../docs/autopilot-integration.md#add-the-paparazzi-module-to-your-own-aircraft))
   and ensure `SENSORAIA_SRC` points to your Sensoraia checkout when compiling.
2. In Paparazzi Center, assign one of the `flight_plans/SENSORAIA/sensoraia_*_demo.xml`
   plans (matching fixed-wing vs. rotorcraft firmware) to your aircraft, save
   your configuration, build target **`nps`**, and execute the matching
   **Sensoraia \<Camera\> NPS** session—the control panel passes `@AIRCRAFT`
   automatically rather than hardcoding an aircraft name.

---

## 7. Troubleshooting

| Symptom | What To Check |
| --- | --- |
| Sensoraia config or panel not listed in Center | Run `./kickstart.sh` and restart Paparazzi Center |
| `USHER` fails to start (`sw/ext/sensoraia/usher`) | Run `make native` (or `make all`) and `./kickstart.sh` in the Sensoraia root |
| `Simulator` reports missing `simsitl` | Click **Build** (target `nps`) for the selected aircraft in Paparazzi Center's **Configuration** tab |
| `local mode is already running` or `/tmp/usher-sim` busy | Stop the previous Center session before launching a new one |
| `--mock-map needs zoom-18 XYZ JPEGs` | Pan over the flight area at zoom 18 in `PprzGCS` so `~/.local/share/paparazzi/pprzgcs/map/GOOGLE/18` is populated |

For protocol architecture and hardware wiring, continue with
[Autopilot Integration](../../docs/autopilot-integration.md) and the
[FTDI Desk Test](../../docs/desk-test.md).

