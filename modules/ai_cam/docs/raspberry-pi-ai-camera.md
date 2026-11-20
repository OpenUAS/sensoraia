# Use The Raspberry Pi AI Camera

The Raspberry Pi AI Camera is a visible-light camera built around Sony's
IMX500 sensor. Sensoraia can use it for ordinary photographs, compatible
vehicle detection or experimental visual return. Choose one profile at a time.

[AI Camera terms](../../../docs/glossary.md#aicam) |
[Spottercam](../../spotter_cam/docs/spottercam.md) |
[LOOMcam](../../loom_cam/docs/loom-cam.md)

## Connect The Hardware

Power off before connecting or removing the ribbon cable. Use the cable
matching the board's connector; the Pi Zero family needs the smaller
Zero-compatible connection. Seat the cable squarely and close the latch.
Check your board's camera/display connector documentation rather than
assuming similar-looking sockets have the same role.

## Prepare Raspberry Pi OS

The current reference setup uses 64-bit Raspberry Pi OS Trixie and its camera
tools, drivers and matching resources. MORA is not limited to a Pi, but using
this camera on another board depends on a compatible camera stack and requires
that board's own integration and validation. On the reference Pi:

```sh
sudo apt update
sudo apt install imx500-all
```

Use the distribution's camera documentation if that package is unavailable.
Review OS/kernel updates as board maintenance: compatibility needs checking
with the actual installed camera stack, not a dated version copied from a guide.

## Check Camera Discovery

Stop USHER and any other camera client before a standalone check:

```sh
rpicam-hello --version
rpicam-hello --list-cameras
```

Look for the IMX500. With no local display, take a photo without preview:

```sh
rpicam-still --nopreview --timeout 2000 --output ai-camera-test.jpg
```

Open the JPEG on your PC and check that it is sharp, correctly exposed and
oriented as expected. Use the actual reported camera index when multiple
devices are present. A nonempty file alone does not prove image quality.

## Choose A Sensoraia Profile

| Profile | Use It For |
| --- | --- |
| `--aicam` | Ordinary photographs using the OS camera tool |
| `--aicam --native-photo` | Native photographs without a model or `rpicam-still` |
| `--spottercam` | Compatible vehicle detection and same-image results |
| `--spottercam --native-photo` | Native photographs with Spottercam detection disabled |
| `--loomcam` | Experimental visual-return observations and bounded target policy |

All three use the same physical camera. Do not start two owners, and do not
run a standalone camera application beside an active USHER capture profile.
Installing camera packages alone does not select a model or enable detection.

An IMX500 model uses a compatible RPK. A CPU ONNX model is a separate choice;
see [choose a model](../../../docs/onnxruntime.md#choose-a-model). Renaming an
unrelated model does not make its outputs meaningful to the application.

## Deploy And Select A Profile

The full MORA package installs all camera workers ready to run. You do **not**
need to hand-edit systemd service templates to switch between them—simply set
`CAMERA` in `camera.conf` at the Sensoraia root (or pass `CAMERA=<name>` on the
`make` command line):

| Desired IMX500 MORA Mode | `camera.conf` / Command-Line Selection | Resulting Service Command |
| --- | --- | --- |
| **Plain AI Camera photos** (native libcamera, no model) | `CAMERA=aicam` | `usher --aicam --native-photo` |
| **Spottercam** *(default: IMX500 + `nn_vehicles.rpk`)* | `CAMERA=spottercam` | `usher --spottercam` |
| **LOOMcam** ([visual-return homing](../../loom_cam/docs/loom-cam.md#how-the-visual-return-loop-works)) | `CAMERA=loomcam` | `usher --loomcam` |

*(Advanced: if you specifically want OS `rpicam-still` instead of `--native-photo`,
or `--spottercam --native-photo`, you can customize `ExecStart` in
`usher.service.in` directly.)*

From your PC checkout, build, upload, and activate the AI Camera profile in one
step:

```sh
make mora-deploy CAMERA=aicam MORA_SSH_TARGET=air@theatre.local
ssh air@theatre.local 'systemctl --no-pager --full status usher.service'
ssh air@theatre.local 'journalctl -u usher.service -n 50 --no-pager'
```

Adjust the SSH target and `MORA_INSTALL_DIR` for your installation. Deployment
rebuilds the ARM64 package on the PC, stages it on MORA, checks dependencies
and restarts USHER. The packaged service uses its configured UART; send a
camera-2 request from the connected flight controller and inspect the saved
image under the service account's `~/Pictures/sensoraia/`. Filenames begin
with `a`, and existing photographs are not overwritten. See the complete
[deployment procedure](../../../docs/README.md#deploy-to-mora) for setup,
password handling and recovery.

## Native Photographs

For photographs without object recognition, select either native photo mode:

```sh
./usher --aicam --native-photo
./usher --spottercam --native-photo
```

Both use the same libcamera capture and JPEG code. The AIcam selection starts
`bin/aicam`; the Spottercam selection starts `bin/spottercam` without a model.
Neither starts `rpicam-still`, loads a model or produces detection results.
USHER also skips SODA analysis for these photo-only captures.
The IMX500's previously retained network is cleared from volatile sensor state;
its model file is unchanged and a later detection session loads it again.

Each request waits for a new frame and saves a 2028x1520 JPEG at quality 90
with 4:2:0 sampling and sensor timing evidence. The camera and storage determine
the available rate. Use the [camera speed test](../../../docs/camera-speedtest.md)
to measure your setup. A failed native capture is not replayed through another
backend. `--native-photo` cannot be combined with mock capture or detector
settings.

| Option | Purpose |
| --- | --- |
| `--aicam` | Select ordinary AI Camera photographs; without `--native-photo`, use `rpicam-still` |
| `--spottercam` | Select the same IMX500 for vehicle detection, or photo-only capture when combined with `--native-photo` |
| `--native-photo` | Use the native libcamera worker without a model, detector, SODA or `rpicam-still` |
| `--speedtest` | Take one warm-up and ten timed photographs, report the observed rate, then exit |
| `USHER_AI_CAM_TIMING=1` | Log native acquisition and JPEG-save times; useful for diagnosing capture latency |

If the native worker fails during a request, that request fails without an
automatic replay. A later request can start a replacement worker after the old
one has exited. Check the service journal and storage space if failures repeat.

For a standalone photograph, stop other camera owners and choose a new absolute
output path:

```sh
./bin/aicam --probe
./bin/aicam --capture /home/air/Pictures/aicam-photo.jpg
./bin/spottercam --photo-only --probe
./bin/spottercam --photo-only --capture /home/air/Pictures/spotter-photo.jpg
```

Run these from `/home/air/sensoraia` on MORA, after stopping USHER if it owns
the camera. `--probe` checks that a frame is available without saving a JPEG;
`--capture` saves one new file and exits. Restore the service afterward.
The target needs its compatible libcamera runtime, IMX500 driver and Raspberry
Pi camera tuning files; no model is needed for native photo mode.

For a short throughput measurement, follow the
[camera speed test](../../../docs/camera-speedtest.md), selecting either
`./usher --aicam --native-photo --speedtest` or
`./usher --spottercam --native-photo --speedtest`. It stops and restores the
service manually around the standalone measurement and retains the test photographs.

## Capture Through USHER

After the board/sensor check, select the intended backend in your reviewed
service configuration and follow [deployment](../../../docs/README.md#deploy-to-mora).
For a separately managed live bench session with no other owner:

```sh
./usher --aicam --debug
```

Connect the prepared FC/UART integration and request one photo. Check both
the saved JPEG and its available flight metadata. Use the
[capture guide](../../../docs/usher-camera-pipeline.md) for service diagnostics
and result interpretation.

## If Something Does Not Work

Check cable seating and power before software changes. A busy-camera error
often means another application owns it. For model failures, inspect the
module's descriptor/export requirements rather than switching randomly
between RPK, ONNX and camera-tool configuration files.

Camera mode, crop, delivered dimensions and mounting affect geometry. A
nominal product field of view is not a calibration of your installed camera.
