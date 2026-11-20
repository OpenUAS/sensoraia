# Camera Speed Test

Measure completed still-image throughput with USHER's built-in `--speedtest`.
It uses the selected camera backend, retains its photographs and exits.
There is no separate speed-test executable to install.

## Prepare The Target

Build on the development PC and follow [deployment](README.md#deploy-to-mora)
to install the current ARM64 release on MORA. Never compile on MORA. For a
PC-attached camera, build with `make native` and run from the checkout instead.
Use the camera's normal settings, output storage and connection. Ensure enough
free disk space for eleven images per run and any configured companion files.

Only one process may own the camera and USHER endpoint. On MORA, connect over
SSH, note whether `usher.service` is active, and stop it before testing:

```sh
ssh air@theatre.local
systemctl is-active usher.service
sudo -n systemctl stop usher.service
cd /home/air/sensoraia
```

Use the actual host and installation directory. If sudo requires a password,
authenticate directly in your terminal; never put the password into a command
or log. Stop any other manually started camera owner too.

## Run A Test

Choose exactly one real image backend:

```sh
./usher --lwircam --speedtest
./usher --aicam --speedtest
./usher --aicam --native-photo --speedtest
./usher --spottercam --speedtest
./usher --spottercam --native-photo --speedtest
./usher --chdk --speedtest
```

Run only the command for your connected camera. No flight controller or UART
connection is needed. Do not combine this mode with daemon/local simulation,
mock images, EARcam, pose logging or LWIR processing options. The worker's
normal settings still apply, so record the settings used for comparisons.
The two `--native-photo` commands use the same direct libcamera capture and
do not load or run an object-recognition model. Plain `--aicam` uses the OS
camera tool, while plain `--spottercam` uses its configured detector.
Set `USHER_AI_CAM_TIMING=1` to report native frame-acquisition and JPEG-save
times separately on standard error.

Capture a log without adding terminal backpressure to the benchmark:

```sh
./usher --lwircam --speedtest > /tmp/lwir-speedtest.log 2>&1
result=$?
cat /tmp/lwir-speedtest.log
printf 'exit=%s\n' "$result"
```

Require exit status zero and ten successful measured captures. Initialization
is reported separately; one additional warm-up capture is excluded. Each run
uses a new `speedtest-*` subdirectory under the backend's normal photo root,
printed in the final report. Original images are retained, not overwritten.
Inspect the saved JPEGs; the benchmark checks the returned path and nonempty
regular file, not complete JPEG decoding or temperature-layer correctness.

Repeat several times to distinguish repeatable throughput from a lucky frame
boundary. Keep all results, including failures, rather than reporting only the
fastest run. On Raspberry Pi, record `vcgencmd get_throttled`,
`vcgencmd measure_temp`, `free -h` and the output filesystem alongside the
application versions and camera settings.

## Interpret The Report

| Report | Meaning |
| --- | --- |
| Practical sustained speed | Ten sequential captures divided by elapsed sequence time, including between-call overhead |
| Average camera-call speed | Ten divided by the sum of timed backend calls |
| Average / fastest / slowest capture time | Latency distribution over those ten calls |
| Fastest single capture | Best individual call; its reciprocal is not demonstrated sustained throughput |
| Initialization | Camera preparation before the warm-up; excluded from the rate |

Backend-call timing includes the work required by that backend to return a
saved image, including encoding/storage where performed there. UART traffic,
the normal flight-trigger loop, USHER EXIF/SODA processing and initialization
are not part of that number. The sustained interval includes between-call
validation/publication work, except work following the final timed return.
Do not treat this short benchmark as an endurance or whole-board headroom test.

The Tiny1-C supplies a 25 fps stream, but fresh-request still capture plus
durable publication can deliver fewer saved images per second. Kernel frame
timestamps are not independently validated exposure timestamps. See
[LWIRcam](../modules/lwir_cam/docs/lwir-cam.md) for startup, freshness, JPEG
quality and exact temperature preservation.

## Measured AI Camera Example

On 2026-11-01, a Raspberry Pi Zero 2 W MORA and its connected IMX500 were tested
with temporary ARM64 binaries and `rpicam-still` unavailable on the process
PATH. Each run saved ten measured 2028x1520 photographs after one excluded
warm-up. Both native selections used the same quality-90 4:2:0 IJG JPEG path,
without a detector model.

| Native Selection | Run 1 Photos/s | Run 2 Photos/s | Run 3 Photos/s |
| --- | --- | --- | --- |
| `--aicam --native-photo` | 1.190 | 1.189 | 1.190 |
| `--spottercam --native-photo` | 1.190 | 1.190 | 1.190 |

All 66 JPEGs decoded at the expected dimensions; their sensor-frame sequences
and timestamps increased within each run. No detection sidecars were produced.
The board reported `throttled=0x0` before and after, with temperatures of
36.5 and 39.7 C. USHER was restored afterward. This is a short capture test,
not an endurance or complete flight-processing measurement.

Earlier on the same board, three plain `--aicam` runs using `rpicam-still`
measured 2.105, 2.121 and 2.103 photos/s. That path used the tool's quality-93
default and zero-shutter-lag mode, so the two results do not isolate camera
access alone: their capture and JPEG pipelines differ.

## Measured MORA Example

On 2026-10-30, the current ARM64 release was deployed to `theatre`, a Raspberry
Pi Zero 2 W Rev 1.0, and tested with the real Tiny1-C. Five independent
`./usher --lwircam --speedtest` runs completed all 50 measured captures plus
five excluded warm-ups. USHER was stopped for the tests and restored afterward.

| Run | Sustained Photos/s | Mean Camera Call, ms |
| --- | --- | --- |
| 1 | 8.255 | 121.103 |
| 2 | 8.387 | 119.184 |
| 3 | 8.427 | 118.612 |
| 4 | 8.464 | 118.102 |
| 5 | 8.402 | 118.977 |

The best observed sustained rate was **8.464 saved photos/s**, about 118 ms
per image. The fastest individual call across these runs was 103.883 ms;
its reciprocal, 9.626 photos/s, is not a sustained result. Initialization was
3.28-3.39 seconds and excluded, as was the extra warm-up capture.

The worker used native V4L2, bundled IJG JPEG, default quality 95 and 4:4:4
sampling, with native inference settings (no model analysis or mock input).
Images were saved to the board's normal microSD-backed ext4 root filesystem,
under `/home/air/Pictures/sensoraia/`. No extra raw companions or statistics
were requested. Durable publication remained enabled. The board reported
`throttled=0x0` before and after, with temperatures of 46.2 and 45.6 C.
All 55 JPEGs were subsequently decoded and their embedded 256x192 temperature
layers validated on the PC; each contained 49,152 valid temperature samples.
No raw companions were retained, so this is not an independent raw-byte
comparison. This is a short board-backed throughput observation, not an
absolute hardware maximum, endurance test or thermal-accuracy qualification.

Board logs are retained at
`/home/air/lwir-speedtest-20261030-dFvUqFOE/`; the logs identify each image
directory. A development-PC evidence copy also retains all images and validation
output.

## Restore The Service

If the service was active before testing, restart it even after a failed test:

```sh
sudo -n systemctl start usher.service
systemctl --no-pager --full status usher.service
journalctl -u usher.service -n 30 --no-pager
```

Leave a previously inactive service inactive. Retain the logs and image paths
with the tested release before removing any benchmark output.
