# LOOMcam

LOOMcam is Sensoraia's visual-return homing application. During the outbound
leg (`START`), it captures downward photographs stamped with the Flight
Controller's barometer altitude, IMU roll/pitch, and magnetometer heading and
links up to 256 reference images per sortie. During the return leg (`STOP`), it
evaluates level frames in RAM and updates waypoint **T1** along the predicted
homing course ray so the aircraft can fly home and settle into an 80 m standby
orbit.

A trained homing network is not included; the PC simulation makes its simulated
homeward vector mode explicit. Vehicle detections and generic features are
never silently treated as navigation directions.

- [Run a manual simulated flight on your PC](docs/loom-cam.md#run-a-manual-simulated-flight-on-your-pc)
- [Prepare and fly a real flight on MORA](docs/loom-cam.md#prepare--fly-a-real-flight-with-loomcam-on-mora)
- [Choose your model and vector frame](docs/loom-cam.md#choose-the-model--vector-frame-loomcam_settingsini)
- [Select or disable the simulated camera profile](docs/loom-cam.md#select-the-simulated-camera-profile)

Build from the Sensoraia root with `make -j32 usher soda loomcam`. For MORA,
follow the shared installation and deployment guide; building alone does not
activate a service.
