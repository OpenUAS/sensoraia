# LWIRcam

Capture supported Tiny1C and TDK thermal cameras. Supported Tiny1C capture
also retains a temperature plane; the supported TDK mode is image-only.

Follow the [capture and measurement guide](docs/lwir-cam.md), then the
[calibration workshop](docs/lwir-calibration.md) for lens, mounting and
independent temperature/location checks.

From the Sensoraia root:

```sh
make -j32 lwircam
bin/lwircam --features
bin/lwircam --help
```

Build/package optional analysis features consistently with USHER. A palette
or a successful build is not temperature or geographic calibration.
