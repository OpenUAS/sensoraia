# Spottercam (`spotter_cam`)

Native vehicle-detection camera module for the Raspberry Pi AI Camera (Sony
IMX500) on MORA (`nn_vehicles.rpk` on-sensor neural network) and CPU ONNX
inference in PC simulation, with ranked five-slot geographic target projection
(`T1..T5`).

Start with the [Spottercam guide](docs/spottercam.md) for IMX500 hardware
setup, model settings (`spottercam_settings.ini`), saved `.objects.ini`
evidence, and the Paparazzi Center simulation.

## Build On Your PC

From the Sensoraia root:

```sh
make usher soda spottercam spottercam-arm64
bin/spottercam --version
```

## Select This Camera

- **MORA deployment (default):** `CAMERA = spottercam` in `camera.conf` (or
  `make mora-deploy CAMERA=spottercam`).
- **Paparazzi Center simulation:** Run `./kickstart.sh`, build aircraft
  `Easystar_3_SPOTTERCAM` (`nps` target), and execute session
  **Sensoraia Spottercam NPS**.
