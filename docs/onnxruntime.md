# Choose And Run An AI Model

Sensoraia can run compatible ONNX models on the CPU, using the same native
inference support across its sensor applications. You choose a separate
model for each job: a vehicle detector, sound classifier and sampling planner
do not interpret the same outputs.

[AI terms explained](glossary.md#ai-and-models) |
[Spottercam](../modules/spotter_cam/docs/spottercam.md) |
[Diplacam](../modules/dipla_cam/docs/diplacam.md)

## Choose A Model

Start with a model exported for the application you want to use. Keep its
graph, descriptor, labels and any deployment metadata together. The filename
extension alone does not establish compatibility.

| Application | The Model's Job |
| --- | --- |
| Spottercam | Return image boxes, classes and scores |
| LOOMcam | Provide features or an explicitly declared return vector; a trained homing model is not included |
| EARcam | Classify the declared microphone sample window |
| LWIRcam | Classify declared display pixels or supported temperature samples |
| Diplacam | Predict one denoising step used by the native measurement planner |

Each application reads its own settings file, such as
`modules/spotter_cam/spottercam_settings.ini`. A typical ONNX assignment is:

```ini
inference_backend=onnx
model=models/assignment/model.onnx
model_descriptor=models/assignment/model.ini
onnx_threads=1
inference_timeout_ms=2000
```

Paths in these settings are relative to the INI file, not your terminal.
Use `--settings /absolute/path/settings.ini` to select another assignment.
Keep the supplied descriptor with the graph: it records the expected input
preparation, tensor layout and graph identity.

Spottercam and LOOMcam can also use a compatible IMX500 `.rpk` with the
physical AI Camera. `inference_backend=auto` selects ONNX for `.onnx` or
IMX500 for `.rpk` on hardware; during PC simulation or saved-image CPU
analysis, Spottercam automatically probes sibling `vehicle_yolo11n_int8.onnx`
and `vehicle_yolo11n_fp32.onnx` exports in `models/` when `model` points to
`models/nn_vehicles.rpk`. CPU inference does not require the camera's neural
processor.

## Check The Assignment Before Capture

From the Sensoraia root, for Spottercam:

```sh
make -j32 spottercam CAMERA_CAPTURE=0
bin/spottercam --check-model --settings /absolute/path/spottercam_settings.ini
```

`CAMERA_CAPTURE=0` builds without physical camera support. It still runs
real model inference. Omit that switch when you need live camera capture.
The model check does not take a photograph or establish detection accuracy.

## Try A Saved Picture

Use an original image representative of the model's intended camera input:

```sh
bin/spottercam --analyze /absolute/path/photo.jpg \
  --settings /absolute/path/spottercam_settings.ini --sequence 1
```

Inspect the result beside the original and read any reported rejection.
Ordinary analysis leaves the input unchanged and writes the application's
result sidecar. The module guide explains its result fields and limitations.

Saved images, MIMIC views and supported live cameras can feed the same model
processing. Their provenance remains different: a real neural result from a
simulated image is still not a real-flight measurement.

## Understand Empty Results And Errors

| Result | What To Do |
| --- | --- |
| Model accepted, no objects found | Keep the empty result; do not invent targets |
| Descriptor/hash mismatch | Restore the graph and descriptor from the same export |
| Unsupported tensor layout or preprocessing | Obtain a compatible export; do not change labels merely to silence the check |
| Runtime library unavailable | Use the matching packaged runtime for this processor and release |
| Inference timeout or failure | Read the diagnostic and stop relying on fresh model results until resolved |

Explicit PC demos can use labelled random observations under their documented
fallback rules. That is a simulation choice, not normal production error
handling. A present invalid model must not silently become a random detector.

## Put The Assignment On MORA

Build and package on the PC using the
[deployment workflow](README.md#deploy-to-mora). The release needs matching
runtime libraries, selected model files and their settings. Review the module's
package selection before activation; a successful local image run does not
measure MORA's shared memory use or sustained processing rate.

Evaluate the model on independent examples resembling its intended use.
Scores are not automatically calibrated probabilities, and changing the
camera, height, lighting or sound environment can change performance.
