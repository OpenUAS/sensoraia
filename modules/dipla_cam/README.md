# Diplacam (`dipla_cam`)

Experimental native GP/diffusion eight-waypoint planner (`P1..P8`) for
rotorcraft adaptive sampling, running over USHER's shared camera-point UART
stream without optical photographs.

Start with the [Diplacam guide](docs/diplacam.md) for field configuration,
model descriptors, the Paparazzi Center rotorcraft simulation, and MORA
deployment.

## Build On Your PC

From the Sensoraia root:

```sh
make usher diplacam diplacam-arm64
bin/diplacam --help
```

## Select This Module

- **Paparazzi Center simulation:** Run `./kickstart.sh`, select aircraft
  `Bebop_DIPLACAM` (`nps` target), click **Build**, and execute session
  **Sensoraia Diplacam NPS**.
- **MORA deployment:** Set `CAMERA = diplacam` in `camera.conf` (or run
  `make mora-deploy CAMERA=diplacam`, or `make mora-deploy MORA_PROFILE=diplacam`
  for a Diplacam-only image).
