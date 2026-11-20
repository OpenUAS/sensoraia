# CHDKbridge (`chdkbridge`)

## What It Does

One focused job: control a CHDK-enabled Canon compact camera cleanly over USB.

CHDKbridge captures full-resolution JPEGs and stores them on your Linux PC or
MORA board, commands optical zoom (`0..255`), and triggers powered lens
retraction before landing—applying your chosen exposure, focus, and quality
parameters on every shot.

Licensed under Apache License 2.0, it serves both as Sensoraia's Canon camera
driver and as a clean standalone USB PTP client where GPL camera tools would be
too restrictive.

## Start With The Guide

Read the [Canon-camera guide](docs/chdkbridge.md). CHDK must already be
installed on the camera's SD card for your exact camera model and firmware;
CHDKbridge runs on MORA or your PC, **not** inside the camera.

## Quickstart

From the Sensoraia root:

```sh
make -j32 chdkbridge
bin/chdkbridge --help
```

To select CHDKbridge on MORA, set `CAMERA = chdk` in `camera.conf` (or run
`make mora-deploy CAMERA=chdk`). For Paparazzi Center simulation, run
`./kickstart.sh`, build `Easystar_3_CHDKCAM` (`nps`), and execute **Sensoraia
CHDKcam NPS**.

## Zero External USB Libraries

CHDKbridge talks directly to the Linux kernel USB device node (`/dev/bus/usb`)
with zero `libusb` or external PTP library dependencies. Ensure the user
account belongs to `plugdev` (`99-chdkbridge.rules` configures this on MORA).
When CHDKbridge starts, it automatically releases any desktop auto-mount of the
camera and claims exclusive USB access while running.