# PANGEA (`pangea`)

Native C++ orthomosaic and georeferenced XYZ-tile reconstruction from saved,
pose-tagged aerial photographs (`SIMPLE` and `GREAT` survey modes), designed
for bounded memory on MORA as well as fast multi-core desktop runs.

Start with the [PANGEA photo-mapping guide](docs/pangea.md) for camera
calibration profiles (`examples/pangea/aicam-2028.ini`), survey configuration,
live incremental mapping with USHER, offline mosaic generation, and
[creating XYZ map tiles for Paparazzi GCS (PprzGCS)](docs/pangea.md#create-map-tiles-for-a-gcs).

## Build On Your PC

From the Sensoraia root:

```sh
make pangea pangea-arm64
bin/pangea --help
```

## Try With Simulated Terrain Photos

Use the **Sensoraia MIMIC PANGEA NPS** session in Paparazzi Center (aircraft
`Easystar_3_MIMIC`) after running `./kickstart.sh`; see the
[MIMIC guide](../../docs/mimic.md) and [PANGEA guide](docs/pangea.md).
