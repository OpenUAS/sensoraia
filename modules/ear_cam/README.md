# EARcam

Search for an alarm using microphone readings and aircraft positions. Start
with the [sound-search guide](docs/earcam-loudest-spot-explained.md) for a
bench check and the supplied simulated flight.

From the Sensoraia root:

```sh
make -j32 earcam
arecord -L
bin/earcam --device default
```

Select your actual ALSA input and keep gain/mounting fixed while comparing
readings. Digital dBFS is not automatically calibrated dB SPL. The resulting
sound-source position is a search aid, not a guaranteed location.

For optional sound classifiers see [model selection](../../docs/onnxruntime.md).
