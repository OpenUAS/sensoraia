# Control A Canon Camera From A Flight Plan

The supported CHDK path can request optical zoom and powered lens retraction
through USHER. These are separate from taking a photograph. The flight
controller sends a request; CHDKbridge performs the camera operation on MORA
and reports its outcome.

[Canon setup](../modules/chdkbridge/docs/chdkbridge.md) |
[Camera-control terms](glossary.md#chdkbridge)

## Before You Start

Use a supported Canon camera with the correct stable CHDK build already
running, matching FC/USHER software and the live CHDK backend. Check one
standalone camera operation on the bench before adding it to a mission.

Other camera backends currently report these controls as unsupported.
Mock camera sessions do not demonstrate mechanical zoom or retraction.

## Use The Supplied Example

The example is
`examples/paparazzi/conf/flight_plans/SENSORAIA/sensoraia_uart_cam_ctrl_demo.xml`.
Install the [Paparazzi examples](../examples/paparazzi/README.md), then inspect
its ticketed camera-control blocks with your aircraft configuration. It is an
integration example, not a complete takeoff-and-landing mission.

Camera actions must not remove the aircraft's normal navigation guidance or
make stable flight depend on a camera responding.

## Choose An Action

| Action | Meaning | Important Distinction |
| --- | --- | --- |
| Zoom 0 | Minimum optical zoom | Not lens retraction |
| Zoom 128 | Middle of the normalized control range | Not a guaranteed magnification ratio |
| Zoom 255 | Maximum optical zoom | Actual steps depend on the camera |
| Retract | Request playback mode while keeping power on | Confirmation is mode evidence, not a lens-position measurement |

A later capture can extend the lens again. Mechanical retraction delay also
depends on the camera's menu setting; see
[powered lens retraction](../modules/chdkbridge/docs/chdkbridge.md#powered-lens-retraction).

## Request Once, Then Read The Result

For flight-plan authors, the existing `digital_cam_uart` interface provides:

```c
camera_request = uart_cam_ctrl_zoom(USHER_CAMERA_MASK_CHDK, 128);
camera_request = uart_cam_ctrl_retract(USHER_CAMERA_MASK_CHDK);
```

These are alternative examples, not a sequence to send together. Keep the
returned `uint32_t` ticket. A zero ticket means the request was not sent;
a nonzero ticket means admitted, not completed.

In XML, use a one-shot statement for the chosen action:

```xml
<call_once
  fun="camera_request = uart_cam_ctrl_retract(USHER_CAMERA_MASK_CHDK)"/>
```

Declare `camera_request` in the plan's variables and include the control header
as shown in the supplied example. Do not resend an action on every control tick.

Read `uart_cam_ctrl_command_pending(camera_request)` while waiting, then
inspect `uart_cam_ctrl_command_result(camera_request)`. A finished wait is
not success. If a later action depends on the camera, provide an explicit
failure branch while preserving appropriate aircraft guidance.

## Understand The Outcome

| Outcome | Meaning |
| --- | --- |
| `NOT_SENT` | No admitted command; inspect configuration/readiness |
| `PENDING` | Still waiting for an outcome |
| `APPLIED` | The backend confirmed the defined effect |
| `BUSY` | The camera could not accept this operation |
| `UNSUPPORTED` | This backend does not implement it |
| `INVALID_VALUE` | The requested value was not accepted |
| `FAILED` | Execution reported failure |
| `UNKNOWN` | Completion could not be established; do not assume nothing happened |

A timeout does not prove the camera never acted. Avoid automatic replay of
an ambiguous physical operation. Check the camera and diagnostics before
deciding what the mission should do next.

## Check Before Onboard Use

Confirm actual camera behaviour, power, USB reliability and the intended
failure response with your setup. A GCS message or successful compilation
does not measure lens position, mechanical timing or flight readiness.
