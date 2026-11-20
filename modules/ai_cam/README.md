# AI Camera (`ai_cam`)

Plain visible-light photo capture on the Raspberry Pi AI Camera (Sony IMX500)
without on-sensor neural inference.

Start with the [Raspberry Pi AI Camera guide](docs/raspberry-pi-ai-camera.md)
for ribbon-cable connection, camera verification, stock `rpicam-still` vs.
native `bin/aicam` photo capture, and USHER integration.

## Build On Your PC

From the Sensoraia root:

```sh
make aicam aicam-arm64
```

## Select This Camera

- **MORA deployment:** Set `CAMERA = aicam` in `camera.conf` (or pass
  `CAMERA=aicam` to `make mora-deploy`).
- **Paparazzi Center simulation:** Run `./kickstart.sh`, build aircraft
  `Easystar_3_AICAM` (`nps` target), and start session **Sensoraia AIcam NPS**.

AIcam, Spottercam, and LOOMcam share the same physical IMX500 camera; run only
one of them at a time.
