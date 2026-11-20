# Sensoraia Glossary

You do not need to learn all these words before using Sensoraia. Start with
the three names below, then come back whenever a guide uses an unfamiliar
term. Each definition explains what the word means **here**, and why it matters
when choosing equipment, running a demo or interpreting a result.

**The short version:** Sensoraia is the project, MORA is the onboard computer,
and USHER is the service that coordinates its sensors. The flight controller
still flies the aircraft.

[Names and responsibilities](glossary.md#names-and-responsibilities) |
[Capture and results](glossary.md#capture-and-results) |
[Position and flight](glossary.md#position-and-flight) |
[Images and timing](glossary.md#images-and-timing) |
[AI and models](glossary.md#ai-and-models) |
[Installation](glossary.md#installation-and-operation) |
[Abbreviations A-Z](glossary.md#abbreviations-a-z)

**Your module:** [AIcam](glossary.md#aicam) | [CHDKbridge](glossary.md#chdkbridge) |
[Diplacam](glossary.md#diplacam) | [EARcam](glossary.md#earcam) | [LOOMcam](glossary.md#loomcam) |
[LWIRcam](glossary.md#lwircam) | [PANGEA](glossary.md#pangea) | [Spottercam](glossary.md#spottercam) |
[MIMIC](glossary.md#mimic) | [Camera Lens](glossary.md#camera-lens)

**A typical journey:** the [flight controller](glossary.md#fc) requests a
[capture](glossary.md#capture). [USHER](glossary.md#usher) asks the selected sensor to do the work,
then keeps the observation with its available [pose](glossary.md#pose) and
[metadata](glossary.md#metadata). A module may also return a detection, position or plan.
The flight plan decides whether and how to act on it.

## Names And Responsibilities

**Sensoraia**
{: #sensoraia .glossary-term }

The camera-compute project as a whole: sensor applications, their supporting
software and the tools to try them on a development PC or use them onboard.
It is not the name of a particular camera or computer.

**MORA**
{: #mora .glossary-term }

The role of the aircraft's onboard companion computer, not a specific board.
Sensoraia's initial resource-constrained reference is a Raspberry Pi Zero 2 W;
another suitable compute board can serve as MORA when its OS, architecture,
drivers, interfaces and resources are supported or adapted. It runs sensor
and processing software separately from the flight controller. MORA is
hardware, not the name of a service to restart.

**USHER**
{: #usher .glossary-term }

The service that coordinates capture and sensor work on MORA, exchanges
requests and results with the flight controller, and records information
alongside observations. Restarting USHER restarts this software, not the
flight controller. You can also run USHER locally on the development PC.

**Companion computer**
{: #companion-computer .glossary-term }

A separate computer that helps the flight controller with jobs such as image
processing and sound analysis. In this project, that computer is MORA.
"Companion" describes its role, not an alternative autopilot.

**FC: Flight Controller; Autopilot**
{: #fc .glossary-term }

The aircraft's flight-control computer. It estimates aircraft state and runs
the control and navigation software. Sensoraia can supply observations or
targets; the FC and its flight plan remain responsible for flying the aircraft.

**GCS: Ground Control Station**
{: #gcs .glossary-term }

The operator's ground-side application for monitoring the aircraft and
sending supported commands. Paparazzi's GCS can display telemetry, photographs'
event markers and waypoints. A point shown on its map is not automatically a
command to fly there.

**Module**
{: #module .glossary-term }

A part of the system with a particular job, such as EARcam's sound search or
LWIRcam's thermal capture. A **Sensoraia module** belongs to the sensing
software; a **Paparazzi FC module** belongs to the aircraft software.
They are not interchangeable installation targets.

**Paparazzi**
{: #paparazzi .glossary-term }

The open-source autopilot system used by the supplied flight examples. It
includes aircraft software, simulation and ground tools. Sensoraia connects
to it at the communication boundary; Sensoraia is a separate project.

**Sensor**
{: #sensor .glossary-term }

Equipment that measures something about the world. Here that includes visible
and thermal cameras and microphones. Other sensor types are possible extension
ideas, not a promise that a driver for every device is already included.

**SODA**
{: #soda .glossary-term }

Sensoraia's saved-image processing application. USHER can hand a completed
image to SODA for its selected camera-specific processing, review products
or optional lens emulation. SODA is not the camera driver, and it does not
mean every image is automatically analysed by a neural network.

## Capture And Results

**API: Application Programming Interface**
{: #api .glossary-term }

A defined way for software components to request work or exchange information.
An API description explains how to interact with a component; it is not the
same as the end-user command or the physical cable carrying a message.

**Acknowledgement; ACK And NACK**
{: #acknowledgement .glossary-term }

A reply about a request. **ACK** is an acknowledgement; **NACK** is a negative
acknowledgement. Always check what was acknowledged: accepting a request,
saving a file and confirming a waypoint position are different events.
An acknowledgement of admission is not proof of completed sensing.

**Backend**
{: #backend .glossary-term }

The implementation selected to do a job. A **camera backend** handles capture
from a particular source; an **inference backend** selects how a model runs.
For example, choosing an ONNX inference backend does not itself choose which
physical camera supplies the image.

**Camera ID And Camera Mask**
{: #camera-id-and-mask .glossary-term }

A camera ID identifies one sensor slot. A mask selects one or more slots by
setting bits in a number. The protocol has eight addressable slots, but not
every slot has an implemented backend. For example, ID 5 is selected by mask
`0x10` (decimal 16), not mask 5. Selecting several cameras does not prove their
exposures happen at the same instant.

**Capture; Shot; Trigger**
{: #capture .glossary-term }

A **trigger** requests an observation. A **capture** is the sensor work that
obtains it. **Shot** is the camera-oriented name used by the shared interface,
including for some non-image measurements. A request can be rejected or fail;
counting triggers is therefore not the same as counting saved photographs.

**DC_SHOT**
{: #dc-shot .glossary-term }

A Paparazzi camera-event message used for shot numbering and GCS display.
One admitted trigger has one GCS shot number even when it selects several
cameras. It is not the UART protocol's message-type number, nor independent
proof that every selected camera saved an image.

**Completion; Saved; Recorded**
{: #completion .glossary-term }

**Completion** means a particular stage finished. A **saved** result concerns
an image output; **recorded** can acknowledge a non-image observation's
admission. Neither word alone guarantees a valid geographic target or a
finished map. Check the result of the stage you actually need.

**Freshness; Stale Result**
{: #freshness .glossary-term }

How recently the information underlying a result was obtained. A newly
arrived reply can already be stale if it was computed from an old pose or
frame. Freshness limits prevent old information being mistaken for a current
target; they do not improve the accuracy of the original measurement.

**IPC: Inter-Process Communication; Socket**
{: #ipc .glossary-term }

**IPC** is how separate running programs exchange information. A **socket**
is one communication endpoint. For example, Diplacam's local worker socket
is between software processes on one computer; it is not the FC UART wire
protocol or an instruction to change a flight-plan block.

**Protocol; Message Type; Payload**
{: #protocol .glossary-term }

A **protocol** defines how a conversation is encoded and interpreted.
A **message type** identifies the kind of message; its **payload** carries
the associated data. A message-type number is not a camera ID, shot number
or waypoint ID, even if two of those numbers happen to be equal.

**Session; Sortie**
{: #session .glossary-term }

A **sortie** is one aircraft outing. A software **session** is a separately
identified period of work, such as one search or planning run. A session token
helps reject delayed replies from earlier work. A session need not cover an
entire flight, and restarting one need not restart the aircraft.

**Telemetry**
{: #telemetry .glossary-term }

Information reported by the aircraft or another running component, such as
position, status or the waypoint coordinates the FC currently holds. It tells
you what was reported; receiving telemetry is different from sending a command.

**Timeout; Backpressure**
{: #timeout .glossary-term }

A **timeout** ends a wait that has taken too long; **backpressure** means a
component cannot currently accept more work. A busy camera can reject a
request instead of adding it to an ever-growing queue. Retrying is a separate
policy: a timeout does not prove a physical exposure never happened.

**UART: Universal Asynchronous Receiver/Transmitter**
{: #uart .glossary-term }

The serial communication interface used between the FC and MORA. **TX** sends,
**RX** receives and **GND** provides the common electrical reference. For the
documented 3.3 V connection, TX connects to the other device's RX. A
compatible connector shape does not establish compatible voltage levels.

**Valid; Invalid; Unavailable**
{: #validity .glossary-term }

**Valid** means the stated checks for a particular result passed.
**Invalid** means it cannot be used as that result; **unavailable** means the
required information was not supplied. An unavailable temperature is not
zero degrees, and a valid computation is not an accuracy certificate.

**Worker; Asynchronous Processing**
{: #worker .glossary-term }

A component that carries out a job separately from the request-handling loop.
**Asynchronous** means the requester need not wait in place until all work is
done; the result arrives later. It does not mean the result is immediate, nor
that unlimited jobs can be queued.

## Position And Flight

**AGL: Above Ground Level**
{: #agl .glossary-term }

Height above the terrain underneath the aircraft, not height above sea level.
An aircraft 50 m above a hill is at 50 m AGL even when the hill itself is high
above the sea. AGL matters when estimating a camera footprint or a sound
source's distance.

**Attitude: Roll, Pitch And Yaw**
{: #attitude .glossary-term }

The aircraft's orientation. **Roll** is banking, **pitch** is nose-up or
nose-down tilt, and **yaw** is rotation around the vertical axis. Camera
projection needs orientation as well as position: a banked camera generally
sees different ground from a level camera at the same coordinates.

**Coordinates; Local Origin**
{: #coordinates .glossary-term }

Numbers that locate a point within a declared reference system. Latitude and
longitude use angles; a local east/north map uses distances from a chosen
**origin**. An east offset of 20 m has no geographic meaning until you know
that origin. Keep both the reference system and the units with the numbers.

**ENU And NED**
{: #enu-ned .glossary-term }

Two ways to name local coordinate directions: **East, North, Up** and
**North, East, Down**. Their component order and vertical sign differ. In NED,
a positive down value points downward, not upward. Neither is the aircraft's
forward/right body frame.

**Fillet turn**
{: #fillet-turn .glossary-term }

A rounded corner joining two straight flight legs. Rather than stopping or
turning sharply at their meeting point, the aircraft follows a connecting
circular arc. This changes which ground positions a sound-search pass samples.

**Flight Plan; Block; Stage**
{: #flight-plan .glossary-term }

The mission instructions interpreted by the autopilot. In Paparazzi, a
**block** groups a mission activity and a **stage** is a step within it.
Updating a target waypoint's coordinates does not, by itself, select a new
block or restart the current stage.

**Geolocation; Geotag**
{: #geolocation .glossary-term }

**Geolocation** estimates a geographic position, for example where an image
pixel meets the ground. A **geotag** stores geographic information with an
image. The aircraft's position in EXIF is not automatically the position of
every object in that photograph.

**GNSS And GPS**
{: #gnss .glossary-term }

**Global Navigation Satellite System** is the general name for satellite
navigation systems. **Global Positioning System** is one such system; "GPS"
is also commonly used informally for the aircraft's satellite-derived
position. Coordinate precision in a file does not establish position accuracy.

**Heading And Course**
{: #heading-and-course .glossary-term }

**Heading** describes where the aircraft's nose points. **Course** describes
the direction of travel over the ground. Wind can make them different, so
course is not a substitute for camera yaw when projecting an image.

**HOME; HOLD**
{: #home .glossary-term }

Named waypoints in the example flight plans. HOME is the plan's home
reference; HOLD is a configured waiting location where provided. They are
not necessarily the aircraft's current position. Entering a HOLD block does
not automatically mean hovering exactly where a failure happened.

**MSL: Mean Sea Level; Ellipsoid Height; Geoid**
{: #msl .glossary-term }

Different references for altitude. MSL is a sea-level-based reference;
**ellipsoid height** uses a mathematical Earth surface. A **geoid** model
relates these references. For a simple example, ground at 30 m MSL and an
aircraft at 80 m MSL give 50 m AGL. Do not subtract an MSL ground height from
an ellipsoid aircraft height without the appropriate conversion.

**NPS: New Paparazzi Simulator**
{: #nps .glossary-term }

Paparazzi's flight-simulation environment. It runs the aircraft software with
a simulated aircraft so you can exercise a complete command-and-result loop
without a physical flight. A successful simulation is not a measurement of
real sensor accuracy or onboard processing speed.

**Pose**
{: #pose .glossary-term }

Position and orientation together. A useful sensor pose also says when it
applies and which coordinate conventions it uses. The pose at request time
may differ from the pose during exposure because the aircraft keeps moving.

**UTM: Universal Transverse Mercator**
{: #utm .glossary-term }

A family of projected coordinate systems that express position in metres
within numbered zones. UTM is convenient for local distance calculations,
but an easting/northing pair must retain its zone and hemisphere.

**Waypoint; Target**
{: #waypoint .glossary-term }

A named geographic point available to the flight plan. A sensing **target**
may be stored as a waypoint, but detecting something and commanding the
aircraft to fly toward it are separate actions. The flight plan chooses the
navigation behaviour and flight altitude.

**WGS84: World Geodetic System 1984**
{: #wgs84 .glossary-term }

The Earth reference used for the latitude/longitude coordinates in these
workflows. It provides a common geographic reference, not a guarantee that
a reported position is correct to any particular number of metres.

**Units: m, mm, Hz, fps, ms, us And ns**
{: #units .glossary-term }

Metres and millimetres measure distance; 1 m is 1,000 mm. **Hertz**, Hz,
counts cycles or updates per second; **fps** counts frames per second.
Milliseconds, microseconds and nanoseconds are fractions of a second:
1 s is 1,000 ms, 1,000,000 us or 1,000,000,000 ns. A camera's advertised
frame rate is not a promise of the saved-photo or inference rate.

## Images And Timing

**Calibration And Validation**
{: #calibration .glossary-term }

**Calibration** estimates parameters or corrections using reference
measurements. **Validation** checks performance on independent observations.
A calibration file existing, or a fitting program finishing, is not the same
as demonstrating accuracy on new measurements.

**EXIF: Exchangeable Image File Format**
{: #exif .glossary-term }

Structured metadata that can be stored inside a photograph, including
supported location, camera and exposure information. Sensoraia also preserves
specific module-owned evidence records. Editing software may discard metadata;
keep original files when you need their measurement history.
[More about Sensoraia's EXIF.](exif.md)

**Exposure; Shutter Time; Rolling Shutter**
{: #exposure .glossary-term }

An **exposure** is the interval over which a sensor gathers light. **Shutter
time** is its duration. With a **rolling shutter**, different image rows are
exposed at different times. A moving aircraft can therefore introduce blur
or geometric skew even when one pose is stored for the whole image.

**Footprint; FOV: Field Of View**
{: #footprint .glossary-term }

FOV is the angular extent of a camera's view; the **footprint** is the area
of ground that view covers at a particular position, height and orientation.
A wider lens or greater AGL generally covers more ground. Horizontal,
vertical and diagonal FOV are different measurements, not interchangeable
versions of one number.

**Frame; Sequence Number**
{: #frame .glossary-term }

One image from a camera stream. A **sequence number** identifies its place in
that stream and helps associate results with the correct observation. Camera
sequence numbers and FC shot numbers are different identifiers; matching
numeric values alone does not prove two records belong together.

**Ground Plane; Nadir**
{: #ground-plane .glossary-term }

A **ground plane** is a flat surface used to simplify image projection.
**Nadir** means directly downward. A flat-ground calculation cannot account
for every roof, tree or slope; a downward-looking camera can still move away
from nadir when the aircraft banks.

**JPEG, PNG, RGB And Grayscale**
{: #image-formats .glossary-term }

**JPEG** is a commonly compressed photographic image format; **PNG** is a
lossless image format often used for diagrams and masks. **RGB** means red,
green and blue colour channels; **grayscale** stores intensity without colour.
These describe image storage or appearance, not whether the pixels are
calibrated temperatures. JPEG expands to Joint Photographic Experts Group;
PNG to Portable Network Graphics.

**Latency; Jitter**
{: #latency .glossary-term }

**Latency** is delay, for example between requesting a photo and its exposure.
**Jitter** is variation in that delay. File arrival time includes later
processing and transfer, so it is not automatically the exposure time.

**Metadata; Provenance**
{: #metadata .glossary-term }

**Metadata** describes an observation: for example its pose, time, sensor or
model identity. **Provenance** records where it came from and how it was
processed. These help distinguish physical observations, simulated inputs
and derived results without guessing from a filename.

**Observation; Request Pose; Observation Pose**
{: #observation .glossary-term }

An **observation** is what a sensor measured, with its relevant context.
**Request pose** describes the aircraft when the capture was requested;
**observation pose** describes the estimated aircraft pose at sensing time.
The latter requires timing evidence. Missing timing must not be silently
relabelled as a synchronized observation.

**Pixel; Resolution**
{: #pixel-and-resolution .glossary-term }

A **pixel** is one sample in an image grid. Image resolution commonly means
its width and height in pixels; it is not the same as the detail the optics
resolve or the accuracy of a geographic estimate. Enlarging an image creates
more output pixels, not new measured detail.

**Sidecar; Embedded Data**
{: #sidecar .glossary-term }

A **sidecar** is a separate file associated with an image, such as an analysis
INI file. **Embedded data** lives inside the image file itself. Keep required
sidecars with their original image; do not assume every workflow writes them
or that an image editor preserves every embedded record.

**Synchronization; Clock Alignment**
{: #synchronization .glossary-term }

Relating events or clocks so the system can estimate which aircraft pose
belongs to a measurement. This includes limits on timing uncertainty, not
just putting timestamps on both files. At 10 m/s, a 0.1 s timing error alone
corresponds to about 1 m of travel.

**Timestamp; UTC; Monotonic Time**
{: #timestamp .glossary-term }

A **timestamp** says when something happened according to a particular clock.
**UTC**, Coordinated Universal Time, is a shared civil-time reference.
**Monotonic time** measures elapsed time without ordinary wall-clock jumps.
Monotonic values from different computers cannot simply be subtracted as
though their clock origins were the same.

**Uncertainty, Precision And Accuracy**
{: #uncertainty .glossary-term }

**Uncertainty** describes how well a value is known. **Precision** concerns
repeatability or, in file output, the number of digits shown. **Accuracy** is
closeness to an independent reference. More decimal places, a sharper image
or a high model score do not by themselves establish better location accuracy.

## AI And Models

**AI, ML And Neural Network**
{: #ai .glossary-term }

**Artificial Intelligence** is the broad field of software performing tasks
such as recognition. **Machine Learning** learns behaviour from examples.
A **neural network**, or NN, is one kind of learned model. In Sensoraia, a
network's job depends on its contract: detecting objects, classifying sound
or proposing measurements are different tasks.

**Class; Classification**
{: #class-and-classification .glossary-term }

A **class** is a label such as "vehicle" or an acoustic category.
**Classification** chooses or scores categories for an input. It does not
necessarily say where an object is in an image, and it does not automatically
produce a geographic target.

**Confidence; Score; Threshold**
{: #confidence .glossary-term }

A model's **score** expresses support for an output according to that model.
A **threshold** decides which scores to accept. Do not read 0.8 as an 80%
chance of a correct result unless the model's calibration establishes that
interpretation. Computational-validity flags and geographic accuracy are
different quantities again.

**CPU And Neural Accelerator**
{: #cpu-and-accelerator .glossary-term }

The **Central Processing Unit** runs general-purpose programs. A neural
accelerator is specialized hardware for compatible model computations.
Sensoraia's ONNX baseline runs on the CPU; an IMX500 RPK uses the AI Camera's
neural processor. A model file alone does not make an accelerator available.

**Dataset; Ground Truth**
{: #dataset .glossary-term }

A **dataset** is a collection of examples. **Ground truth** is the reference
answer associated with an example, such as a labelled object or a known
synthetic field value. Training and evaluation need suitable labels, rights
and representative inputs; saved imagery is not automatically a ready-to-use
training dataset.

**False Positive And False Negative**
{: #false-positive .glossary-term }

A **false positive** reports something that is not there; a **false negative**
misses something that is. Adjusting a score threshold changes this trade-off.
One successful demonstration does not measure how often either occurs in
different light, terrain, sound or weather.

**Inference And Training**
{: #inference .glossary-term }

**Training** learns model parameters from examples. **Inference** runs the
already trained model on a new input. Selecting an ONNX model and analysing a
photo performs inference; it does not train or automatically improve that model.

**Model; Graph; Checkpoint**
{: #model .glossary-term }

A learned **model** combines computations with trained parameters. An exported
**graph** describes those computations in a runtime-readable form. A
**checkpoint** stores a particular training state. Models are not
interchangeable just because their filenames have the same extension.

**Model Descriptor; Model Contract**
{: #model-descriptor .glossary-term }

The descriptor states which graph and tensor layout an application expects,
including preprocessing and a graph hash. The wider **contract** says what
the inputs and outputs mean. A vehicle detector is not a visual-return model,
and a generic feature vector is not a navigation direction.
[Choose a compatible model bundle.](onnxruntime.md#choose-a-model)

**ONNX And ONNX Runtime; ORT**
{: #onnx .glossary-term }

**Open Neural Network Exchange** is the model format. **ONNX Runtime**, often
abbreviated ORT, is the engine that executes compatible graphs. Sensoraia
applications share native runtime support but select their own models and
settings. ONNX support does not imply support for every custom operator.

**Preprocessing; Normalization; Letterbox**
{: #preprocessing .glossary-term }

Preparing input exactly as a model expects. **Normalization** rescales values;
**letterbox** resizes an image while preserving its aspect ratio and pads the
remaining area. Colour order and interpolation also matter: a plausible-looking
image is not enough to establish the right numerical input.

**FP32, INT8 And Quantization**
{: #quantization .glossary-term }

**FP32** uses 32-bit floating-point numbers. **INT8** uses 8-bit signed integers.
**Quantization** represents values at reduced numerical precision, usually
with scales and offsets. It may reduce model cost, but speed and accuracy
depend on the export and runtime. An INT8 filename alone is not proof of
compatibility, acceleration or unchanged quality.

**RPK**
{: #rpk .glossary-term }

The model-package file used by the IMX500 camera path. It must suit the camera
and application contract; it is not a general-purpose CPU model. An `.onnx`
and an `.rpk` are different deployment choices, not extensions to rename.

**Tensor; Shape; NCHW And NHWC**
{: #tensor .glossary-term }

A **tensor** is a structured array of numbers passed to or from a model.
**Shape** specifies its dimensions. For image batches, NCHW means batch,
channels, height, width; NHWC means batch, height, width, channels. The same
numbers arranged differently can mean a completely different input.

## Installation And Operation

**ABI: Application Binary Interface**
{: #abi .glossary-term }

The low-level rules that let a compiled program use libraries and operating
system facilities. Two ARM64 files can still be incompatible if their runtime
requirements differ. Matching the processor name alone is not a deployment
compatibility check.

**Architecture: x86-64, ARM64 And AArch64**
{: #architecture .glossary-term }

The processor instruction family a binary runs on. **ARM64** and **AArch64**
refer to the 64-bit Arm target used by MORA; **x86-64** is the development-PC
target in the supplied builds. The production source is shared, but the
compiled binaries are different.

**Build; Cross-compile; Deploy**
{: #build .glossary-term }

To **build** is to turn source code into runnable binaries. To **cross-compile**
is to build for another target, such as compiling ARM64 on the PC. To **deploy**
is to upload and activate a prepared release on MORA. Building or packaging
does not deploy it, and deployment can restart USHER.
[Follow the installation path.](README.md)

**CLI; Option; Flag; Environment Variable**
{: #cli .glossary-term }

**Command-Line Interface** means operating a program through typed commands.
An **option** or **flag**, such as `--help`, changes that invocation. An
**environment variable**, such as `AIRCRAFT`, supplies context inherited by
programs. Neither automatically rewrites an application's settings file.

**INI, JSON, CSV And XML**
{: #configuration-formats .glossary-term }

Text formats for different kinds of information. **INI** is an initialization-
file format used for settings; **JSON**, JavaScript Object Notation, stores
structured values; **CSV**, Comma-Separated Values, stores rows of data;
**XML**, Extensible Markup Language, uses named elements, including Paparazzi
flight plans. A JSON file does not require a JavaScript runtime.

**Firmware**
{: #firmware .glossary-term }

Software running inside a device such as a camera or flight controller.
Uploading a Sensoraia application to MORA does not replace Canon firmware,
install CHDK on its SD card or flash a new autopilot image.

**Headless; SSH**
{: #headless .glossary-term }

**Headless** operation uses no locally attached display or keyboard. **Secure
Shell**, SSH, provides a remote terminal. A command typed in an SSH session
runs on the remote computer; a command typed in your ordinary PC terminal
runs on the PC. This distinction matters for build and deployment commands.

**Log; Diagnostic; Exit Status**
{: #log .glossary-term }

A **log** records what software reports while running. A **diagnostic** explains
a condition or failure. **Exit status** is the final numeric result of a
command, normally zero for success. Output text alone may describe an
intermediate stage; inspect the saved result and final status too.

**Native**
{: #native .glossary-term }

Compiled machine-code execution, here primarily C and C++. In a build command,
"native build" also means a build for the current PC rather than a
cross-build. Native does not mean real-camera input, hardware acceleration,
no dependencies or demonstrated whole-board memory headroom.

**Package; Release**
{: #package .glossary-term }

A prepared collection of application binaries and required settings, models,
runtime libraries and notices. A **release** identifies a particular collection
for distribution or deployment. A package on the PC is not already activated
on MORA.

**Absolute Path; Relative Path; Working Directory**
{: #paths .glossary-term }

An **absolute path** starts at the filesystem root, such as
`/home/air/sensoraia/bin/earcam`. A **relative path** needs a reference
directory. Shared model-settings paths are relative to their INI file;
ordinary command arguments may instead be relative to the shell's **working
directory**. Moving a settings file can therefore change what its paths select.

**RAM; CMA; MiB**
{: #ram .glossary-term }

**Random Access Memory** is working memory shared by the operating system,
applications and device buffers. The Pi Zero 2 W reference board has 512 MB
total; other MORA boards may have different capacities.
Linux's **Contiguous Memory Allocator**, CMA, reserves contiguous memory for
devices such as cameras. **MiB**, mebibytes, counts bytes in powers of two.
A small executable file does not imply a small runtime memory requirement.

**SDK; Toolchain; Sysroot**
{: #sdk .glossary-term }

A **Software Development Kit** supplies build-time components for a target.
A **toolchain** includes its compiler and related tools; a **sysroot** provides
target headers and libraries. Sensoraia prepares these on the development
PC for cross-compilation. They are not the application runtime package
deployed to MORA.

**Service; systemd; Journal**
{: #service .glossary-term }

A **service** runs under operating-system supervision rather than only inside
an interactive terminal. On MORA, **systemd** manages `usher.service`; its
**journal** contains service logs. Editing a service configuration is different
from rebuilding the application or changing a camera's firmware.

**SHA-256; Hash; Checksum**
{: #sha256 .glossary-term }

A **hash** is a fingerprint computed from file contents; **SHA-256** is Secure
Hash Algorithm 256-bit. A checksum comparison can reveal changed or mismatched
files. It does not establish trustworthy authorship by itself: an attacker
could replace both an untrusted file and its untrusted expected hash.

**USB, UVC, PTP And CSI**
{: #usb-and-csi .glossary-term }

**Universal Serial Bus**, USB, connects devices such as microphones and thermal
cameras. **USB Video Class**, UVC, and **Picture Transfer Protocol**, PTP, are
different device protocols carried over USB. The **Camera Serial Interface**,
CSI, is the ribbon-cable camera connection used by the AI Camera. These are
not substitutes for the FC-to-MORA UART link.

## AIcam

AIcam is the visible-camera path using the Raspberry Pi AI Camera. Spottercam
and LOOMcam can use the same physical camera for different processing jobs.
[AI Camera setup and operation.](../modules/ai_cam/docs/raspberry-pi-ai-camera.md)

**IMX500**
{: #imx500 .glossary-term }

Sony's image sensor with an integrated neural processor, used in the Raspberry
Pi AI Camera. Its hardware can run a compatible RPK while capturing images.
Ordinary image capture does not automatically enable a particular detector.

**libcamera; rpicam-still**
{: #libcamera .glossary-term }

**libcamera** is the camera-control framework used by the native camera workers.
**rpicam-still** is a separate Raspberry Pi camera application used by the stock
capture path. They are not two names for the same executable. A physical
camera still needs one capture owner at a time.

**Sensor Mode; Crop; Binning**
{: #sensor-mode .glossary-term }

A sensor mode chooses how the camera reads its image. A **crop** selects part
of the sensor; **binning** combines samples to produce a lower-resolution
readout. Changing these can change image geometry, so a calibration for one
mode must not be assumed valid for another merely by matching the filename.

## CHDKbridge

CHDKbridge controls supported Canon cameras from MORA or the PC.
[Canon-camera setup and commands.](../modules/chdkbridge/docs/chdkbridge.md)

**CHDK: Canon Hack Development Kit**
{: #chdk .glossary-term }

A separately installed software add-on running inside a supported Canon
camera. It adds scripting and control functions. CHDK must match the camera
model and firmware; installing Sensoraia on MORA does not install CHDK on
the camera.

**CHDKbridge**
{: #chdkbridge-controller .glossary-term }

Sensoraia's native controller that communicates with camera-side CHDK over
USB/PTP. It runs on MORA or the PC, not inside the Canon camera. USHER's
CHDK backend invokes it for the selected camera work.

**Direct Download**
{: #direct-download .glossary-term }

The optional CHDK capture path that transfers JPEG data directly over USB
when the camera port supports it. Ordinary capture saves the photo on the
camera's SD card and downloads that file. CHDK support alone does not prove
the direct-JPEG capability is available.

**Optical Zoom; Normalized Zoom**
{: #optical-zoom .glossary-term }

**Optical zoom** changes the lens's magnification. CHDKbridge's normalized
0..255 control maps onto the camera's available zoom steps; 128 is roughly
the midpoint of that control range, not a guaranteed 2x magnification.
This is different from cropping or enlarging pixels digitally.

**Playback Mode; Lens Retraction**
{: #playback-mode .glossary-term }

Playback is the camera mode used for reviewing images rather than taking
them. Requesting it can let the powered Canon camera retract its lens.
Mode confirmation does not measure physical lens position or retraction
time; a later capture can extend the lens again.

## Diplacam

Diplacam proposes where to measure next using accumulated knowledge of a
field, rather than following only a fixed coverage pattern.
[The measurement-to-plan guide.](../modules/dipla_cam/docs/diplacam.md)

**Belief; Mean; Variance**
{: #belief .glossary-term }

The planner's current estimate of the field and its uncertainty. The **mean**
is the estimated value at each grid cell; **variance** describes uncertainty
in that estimate. Belief is not the complete field truth: it changes as
measurements arrive.

**CMA-ES: Covariance Matrix Adaptation Evolution Strategy**
{: #cmaes .glossary-term }

A numerical search method that improves candidate plans by evaluating and
adapting a population of proposals. Diplacam offers it as an explicitly
selected comparison mode, not a silent substitute for a failed ONNX model.
It is unrelated to Linux camera-memory **CMA**.

**DDIM: Denoising Diffusion Implicit Model**
{: #ddim .glossary-term }

A sampling method that repeatedly refines a noisy candidate using a diffusion
model. Diplacam runs twenty native denoising steps. One ONNX call predicts
noise for one step; it is not the entire eight-point planner by itself.

**Diffusion Planner**
{: #diffusion-planner .glossary-term }

A learned planner that turns an initial noisy candidate into proposed
measurement locations, conditioned on the current belief and aircraft
position. "Diffusion" here describes the model's generation process, not
physical spreading of a measured substance.

**GP: Gaussian Process; Posterior**
{: #gp .glossary-term }

A statistical model that relates nearby field values and maintains an estimate
with uncertainty. After incorporating a measurement, the updated estimate is
the **posterior**. Measuring one area can change expectations nearby, but
does not turn unmeasured cells into independently observed truth.

**Informative Path Planning**
{: #informative-path .glossary-term }

Choosing measurement locations to learn useful information efficiently,
rather than merely visiting evenly spaced positions. The research objective
combines expected information and travel cost. Whether a particular model
achieves useful plans needs evaluation in the intended conditions.

**P1-P8; Plan Generation; Snapshot**
{: #p1-p8 .glossary-term }

The eight waypoints refreshed by the Diplacam example. A **generation**
identifies one completed native plan; a **snapshot** keeps its eight points
together during retrieval. Publishing eight ordinary GCS waypoint moves is
not an atomic eight-point update of the FC.

**Scalar Field; Synthetic Observation**
{: #scalar-field .glossary-term }

A map with one numerical value at each location. Diplacam's supplied CSV
stands in for sensor returns: only the current footprint is sampled into the
GP belief. It is not an RGB image, a full spectral-image cube or an already
implemented driver for a physical sensor.

**Training Range; Predicted Z**
{: #training-range .glossary-term }

The conditions represented during model training. Diplacam's bundled model
uses XYZ internally and was trained for 10..40 m heights; the current demo
commands 2 m and publishes only XY. **Predicted Z** is discarded as a flight
command. Correct software execution at 2 m does not establish planning
quality outside the model's training range.

## EARcam

EARcam searches by sound: it associates microphone measurements with aircraft
positions to estimate where an alarm source is located.
[Continue with the sound-search guide.](../modules/ear_cam/docs/earcam-loudest-spot-explained.md)

**ALSA: Advanced Linux Sound Architecture**
{: #alsa .glossary-term }

The Linux sound system that supplies microphone samples to EARcam. An ALSA
device name selects an audio input; it does not select an acoustic detection
algorithm.

**Band-pass filter**
{: #band-pass-filter .glossary-term }

A filter that keeps a chosen range of sound frequencies and reduces sounds
outside it. EARcam uses this to concentrate on the alarm's frequency range
rather than treating all microphone noise as equally useful.

**Contrast: EARcam tone contrast**
{: #contrast .glossary-term }

A comparison between the strongest detected tone and the energy across the
selected frequency band. A distinct alarm tone has stronger contrast than
broad noise. This is an audio quality check, not visual image contrast.

**Decibels: dB, dBFS And dB SPL**
{: #decibels .glossary-term }

A logarithmic way to compare signal levels. The reference matters: **dBFS**
means decibels relative to a digital recording's maximum level; **dB SPL**
means sound-pressure level relative to a standard physical pressure.
A microphone's dBFS reading is not automatically a calibrated dB SPL reading.
Keep the microphone gain consistent when comparing EARcam measurements.

**Envelope**
{: #envelope .glossary-term }

A smoothed description of how loud a signal is over time. It bridges short
gaps between an alarm's pulses so one brief quiet interval does not make the
source appear to vanish.

**Gauss-Newton**
{: #gauss-newton .glossary-term }

A repeated-improvement method for fitting a mathematical model to
measurements. EARcam uses a fitted sound model to estimate a source position
from readings taken at different aircraft positions. It does not simply choose
the coordinates of the single loudest reading.

**Goertzel filter**
{: #goertzel-filter .glossary-term }

An efficient way to measure how much of a particular tone is present in a
sound recording. EARcam examines several nearby frequencies to accommodate
variation in the alarm tone.

**Hampel filter And MAD: Median Absolute Deviation**
{: #hampel-filter .glossary-term }

A method for spotting measurements that differ unusually strongly from nearby
ones. MAD measures typical spread around the median, so one extreme spike
does not set the scale for everything else. EARcam uses this kind of robust
filtering to reduce the effect of isolated bad readings.

**Huber weights**
{: #huber-weights .glossary-term }

A way to reduce the influence of measurements that disagree strongly with a
model, without giving up on the whole collection. In EARcam's position fit,
this helps prevent a few unusual sound readings from dominating the answer.

**motionSCOUT**
{: #motionscout .glossary-term }

MSA's personal motion-alarm device, used as the sound source in the EARcam
search example. Its pulsed alarm gives the detector a recognizable tone
pattern to look for. It is the target sound source, not a component running
on MORA.

**Sound picture**
{: #sound-picture .glossary-term }

The JPEG heat map produced after an EARcam search. It makes the sound-search
result easy to inspect and associate with flight metadata. It is a computed
visualization, not a photograph taken by a microphone.

**PCM: Pulse-Code Modulation; Sample Rate**
{: #pcm .glossary-term }

A representation of recorded sound as regularly spaced numerical samples.
The **sample rate** counts samples per second, not how often a position
estimate is produced. An audio model must receive the declared rate and
window length; changing only a file's label does not resample its audio.

**RMS: Root Mean Square**
{: #rms .glossary-term }

A measure of signal level across a window of samples. EARcam's raw RMS level
and an optional classifier's probability describe different things. A loud
sound need not be the desired alarm, and a high classifier score is not a
calibrated sound-pressure measurement.

**Star Refinement**
{: #star-refinement .glossary-term }

An acoustic-search pattern with passes in several directions around a
candidate location. The different viewing positions help refine the sound
source fit; it is not a new type of microphone. The flight plan still owns
the actual path and its constraints.

## LOOMcam

LOOMcam is the experimental visual-return application. Its model integration
and bounded target policy are present; a trained homing network is not
included. [Visual-return settings and workflow.](../modules/loom_cam/docs/loom-cam.md)

**Features; Embedding**
{: #features .glossary-term }

Numbers produced by a network to describe an input. An **embedding** arranges
descriptions so they can be compared in the intended feature space.
Features from different models may not be comparable. A feature vector alone
does not specify a return direction or a geographic position.

**Homing; Visual Return**
{: #homing .glossary-term }

Using visual information to help identify a direction toward a known home
region.

**Reference Photographs; Outbound Phase**
{: #reference-photos .glossary-term }

Images retained during the outward part of the example flight for possible
later visual comparison. References carry their available capture and model
evidence. Keeping these photos is not proof that a model can reliably match
them from a different viewpoint.

**Return Vector; Vector Frame**
{: #return-vector .glossary-term }

A declared model output describing a direction toward the return spot. Its
**frame** states whether components mean Earth `north_east`, aircraft
`body_forward_right` (rotated into Earth coordinates using the photo's EXIF IMU
roll/pitch and magnetometer heading), or `simulated` (an explicitly labelled
homeward vector seeded by the photo's neural features for testing before a
model is trained). LOOMcam checks and converts the declared frame; it never
guesses the meaning from two raw numbers.

**T1; Look-ahead; HOME Protection Radius**
{: #loom-t1 .glossary-term }

LOOMcam's example return waypoint, initially at HOME. **Look-ahead** places a
target along an accepted homing ray; the **protection radius** (`60 m` by
default) locks `T1` at HOME once the aircraft arrives so it can settle into a
smooth `80 m` standby orbit without chasing a point underneath its belly.
These checks bound target updates; they do not demonstrate visual homing
accuracy by an untrained model.

## LWIRcam

LWIRcam captures supported thermal cameras. Measurement and geolocation
depend on the camera's available data and the enabled analysis features.
[Capture guide](../modules/lwir_cam/docs/lwir-cam.md) |
[Thermal calibration guide](../modules/lwir_cam/docs/lwir-calibration.md)

**LWIR: Long-Wave Infrared**
{: #lwir .glossary-term }

The infrared wavelength region used by these thermal cameras. A displayed
thermal image represents detected infrared information; its colour palette
is not visible-light colour, and brightness is not automatically a calibrated
temperature reading.

**Radiometric; Temperature Plane; Image-only**
{: #radiometric .glossary-term }

**Radiometric** data supports numerical temperature-related measurements.
A **temperature plane** contains those per-pixel values separately from the
display image. Supported Tiny1C capture preserves this plane; the supported
TDK image-only mode does not provide it. A colourful thermal JPEG alone
cannot supply missing temperature samples.

**Kelvin64**
{: #kelvin64 .glossary-term }

The preserved Tiny1C temperature encoding: kelvin multiplied by 64. For a
valid sample, divide by 64 to obtain kelvin, then subtract 273.15 for Celsius.
Reserved invalid codes are not cold or hot temperatures and must not be
converted into ordinary measurements.

**Emissivity; Reflected Temperature**
{: #emissivity .glossary-term }

**Emissivity** describes how effectively a surface emits thermal radiation
compared with an ideal emitter. **Reflected temperature** accounts for thermal
radiation reflected from the surroundings. A shiny surface can show reflected
heat rather than its own temperature; visual brightness cannot resolve that
ambiguity on its own.

**FFC And NUC**
{: #ffc-nuc .glossary-term }

**Flat-Field Correction** and **Non-Uniformity Correction** address uneven
response across a thermal sensor. A camera may use a shutter/reference event
as part of its correction. This is different from lens calibration and does
not by itself establish absolute-temperature accuracy. Do not assume a
software readback records every correction event.

**Intrinsics; Extrinsics; Boresight**
{: #intrinsics-extrinsics .glossary-term }

**Intrinsics** describe the lens and sensor geometry, including focal lengths,
image centre and distortion. **Extrinsics** describe how the camera is mounted
relative to the aircraft. **Boresight** concerns the viewing-axis alignment.
Both lens geometry and mounting matter when placing a hotspot on a map.

**Palette; Display Window; Percentiles**
{: #palette .glossary-term }

A **palette** converts displayed levels into colours. A **display window**
chooses the temperature range mapped into those levels. **Percentiles** can
choose a range from the distribution of valid values. These can make a scene
easier to inspect without changing the preserved temperature samples.

**ROI: Region Of Interest**
{: #roi .glossary-term }

A selected part of an image or sensor plane. LWIRcam can measure a rectangular
ROI, point or line using native sensor coordinates. Rotating the displayed
JPEG does not rotate those measurement coordinates; check the coordinate
convention before selecting a region.

**Hotspot; Ground Projection**
{: #hotspot .glossary-term }

A region meeting the thermal detector's selection conditions.
**Ground projection** estimates where its image position intersects the
assumed ground using calibration and observation pose. It is an estimated
location, not proof that the region is a fire or that a surveyed error
requirement has been met.

**Correction Profile; Hold-out Check**
{: #thermal-profile .glossary-term }

A **correction profile** records measured corrections for a specified thermal
device and configuration. A **hold-out check** uses reference measurements
not used to fit those corrections. Lens, mounting and temperature checks solve
different problems; one successful calibration does not replace the others.

## PANGEA

PANGEA, **Photo-Assisted Neighboring Geographic Edge Assembly**, turns saved photographs and their supported positioning information
into photo maps. It is a separate processing job, not a prerequisite for
every capture. [Photo-map workflows.](../modules/pangea/docs/pangea.md)

**Georeferenced**
{: #georeferenced .glossary-term }

Associated with a geographic coordinate system so image positions can be
placed on a map. A georeferenced output can still have errors from pose,
timing, lens assumptions or terrain; the presence of coordinates is not a
survey-accuracy certificate.

**GSD: Ground Sampling Distance**
{: #gsd .glossary-term }

The ground distance represented by an image pixel, often expressed in
centimetres per pixel. A map at 10 cm/pixel does not automatically have
10 cm positioning accuracy or recover detail finer than its source images.

**Mosaic; Orthomap**
{: #mosaic .glossary-term }

A **mosaic** combines several images into one visual map. An **orthomap**
places imagery in a map-like, top-down geometry. PANGEA's reference-plane
workflows do not reconstruct every terrain height, so elevated objects
can still be displaced or show seams.

**Seam; Blending; Exposure Compensation**
{: #seam-and-blending .glossary-term }

A **seam** decides where overlapping source images contribute. **Blending**
smooths their visual transition; **exposure compensation** reduces brightness
differences. These improve appearance but do not correct all geometric
misalignment or establish absolute colour/temperature calibration.

**SIMPLE And GREAT**
{: #simple-great .glossary-term }

Names of PANGEA's two completed-survey modes. SIMPLE uses a simpler compositing
approach; GREAT adds graph-cut seam selection and multiband blending.
They describe processing choices, not formal accuracy grades. The separate
live photo-map workflow has its own input and resource rules.

**Coverage; Gap Filling**
{: #coverage .glossary-term }

**Coverage** says which ground was represented by accepted photographs.
**Gap filling** makes output visually complete using supported source-colour
completion. Filled pixels are not extra photographs or newly measured
terrain. An output with no blank pixels need not have complete observed
coverage.

**Incremental Map; Provisional Revision; Finalize**
{: #incremental-map .glossary-term }

An **incremental** map updates as accepted photos arrive. A **provisional
revision** is a coherent intermediate product, not a complete survey.
**Finalizing** ends input collection and produces the finished session output;
it does not independently validate geographic accuracy.

**Manifest; Checkpoint; Resume**
{: #manifest .glossary-term }

A **manifest** identifies inputs or packaged files. A **checkpoint** records
completed work and the identities needed to reuse it. **Resume** continues a
matching interrupted job rather than starting from zero. Editing committed
inputs can invalidate that match; a checkpoint is not permission to mix
unrelated surveys.

**XYZ Tiles; Tile Pyramid; Zoom Level**
{: #xyz-tiles .glossary-term }

A web-map layout addressing square image tiles by zoom level and X/Y indices.
A **tile pyramid** stores several zoom levels for efficient viewing. Higher
zoom shows a finer grid; it does not create finer source detail. XYZ here
means tile addresses, not a three-dimensional point or elevation model.

## MIMIC

MIMIC is **Map-tile Imagerie Modeling in Interactive Camera-sim**: the
map-based camera simulator used in the PC examples.
[Make camera views from cached imagery.](mimic.md)

**Map Cache; Map Tile**
{: #map-cache .glossary-term }

A **map tile** is one small image from a map provider. The **cache** is the
local collection already downloaded by the GCS. MIMIC uses the configured
cached aerial imagery to render a camera view; displaying the aircraft in
the GCS does not automatically prove every tile in its camera footprint
is available.

**Mock; Fake Camera; Simulated Detection**
{: #mock .glossary-term }

A **mock** replaces a particular component for a test. A fake camera can
supply saved or rendered images while the real native model still runs.
A **simulated detection** instead fabricates a detector result under an
explicit demo policy. These are different substitutions; a mock JPEG does
not mean every later processing stage is fake.

**Rendered Camera View**
{: #rendered-view .glossary-term }

An image generated from the simulated camera's pose and ground imagery.
Turning or banking the aircraft changes what it sees. MIMIC's ground texture
is not a physical exposure, and visible aerial imagery rendered for a thermal
path is not a temperature measurement.

**MORA-in-the-loop; Camera-in-the-loop**
{: #hardware-in-loop .glossary-term }

In **MORA-in-the-loop**, a simulated flight exchanges requests with USHER on
the real board. **Camera-in-the-loop** includes supported physical capture
hardware. These test different boundaries; real capture paired with simulated
pose does not establish real-flight geolocation accuracy.

**Random Seed**
{: #seed .glossary-term }

A starting value that makes a pseudo-random demo repeatable under the same
conditions. It helps compare software behaviour across runs. Seeded random
targets are still invented targets, not observations made by a trained
detector.

## Camera Lens

The optional lens emulator adds declared optical and shutter effects to
supported MIMIC imagery while retaining the clean source.
[Lens profiles and examples.](camera-lens.md)

**Lens Distortion; Fisheye**
{: #distortion .glossary-term }

Lens geometry that bends the relation between a scene direction and an image
position. A **fisheye** deliberately uses a wide-angle projection with strong
distortion. Correct projection needs that lens's model, not just a wider
crop of a pinhole image.

**Pinhole Model; Ray**
{: #pinhole .glossary-term }

A simplified geometric camera model. A **ray** is an imagined line from the
camera through an image pixel; intersecting it with the assumed ground
locates that pixel's footprint. The model describes geometry, not measured
focus, lens blur or actual shutter timing.

**Lens Profile; Derivative Image**
{: #lens-profile .glossary-term }

A **lens profile** declares the emulation's optical and exposure parameters.
A **derivative** is the processed image made from an original. Labelling an
emulated photo preserves its meaning; it does not turn assumed parameters
into a calibrated physical camera.

## Spottercam

Spottercam detects objects, with the supplied application focused on vehicles.
Its image detections and geographic target updates are separate steps.
[Vehicle-search guide.](../modules/spotter_cam/docs/spottercam.md)

**Bounding Box; Detection**
{: #bounding-box .glossary-term }

A **bounding box** encloses a detected object in image coordinates.
Detection reports a class, box and score. It does not by itself locate that
object on Earth; geographic projection also needs the corresponding camera
geometry, pose and ground assumptions.

**IoU And NMS**
{: #iou-nms .glossary-term }

**Intersection over Union** measures overlap between two boxes: overlap area
divided by their combined area. **Non-Maximum Suppression** reduces competing
boxes for the same apparent object. These operate on image detections, not
on the aircraft's navigation waypoints.

**T1-T5; Best-Five Targets**
{: #spotter-t1-t5 .glossary-term }

Five target waypoints in the Spotter demo, initially at HOME. The native store
fills empty slots, then replaces the lowest score only with a strictly
higher one. These are not LOOMcam's single return target T1, and moving
them does not automatically interrupt the survey.

**Zero Detections; Analysis Failure**
{: #zero-detections .glossary-term }

**Zero detections** means a model ran successfully without accepted objects.
**Analysis failure** means the result could not be produced or trusted.
They are not interchangeable. A genuine empty result must not trigger
fabricated detections; the explicit random spotting policy belongs only to
the simulation harness's documented fallback cases.

## Abbreviations A-Z

Each link leads to the explanation and its practical meaning above. Names
such as MORA, USHER and SODA are project names; their roles are more useful
than an invented expansion of their letters.

| Term | Meaning Or Place To Start |
| --- | --- |
| [ABI](glossary.md#abi) | Application Binary Interface |
| [ACK / NACK](glossary.md#acknowledgement) | Acknowledgement / negative acknowledgement |
| [AGL](glossary.md#agl) | Above Ground Level |
| [AI / ML / NN](glossary.md#ai) | Artificial Intelligence / Machine Learning / neural network |
| [ALSA](glossary.md#alsa) | Advanced Linux Sound Architecture |
| [API](glossary.md#api) | Application Programming Interface |
| [ARM64 / AArch64](glossary.md#architecture) | The 64-bit Arm build target |
| [CHDK](glossary.md#chdk) | Canon Hack Development Kit |
| [CLI](glossary.md#cli) | Command-Line Interface |
| [CMA](glossary.md#ram) | Linux Contiguous Memory Allocator |
| [CMA-ES](glossary.md#cmaes) | Covariance Matrix Adaptation Evolution Strategy |
| [CPU](glossary.md#cpu-and-accelerator) | Central Processing Unit |
| [CSI](glossary.md#usb-and-csi) | Camera Serial Interface |
| [CSV](glossary.md#configuration-formats) | Comma-Separated Values |
| [dB / dBFS / dB SPL](glossary.md#decibels) | Decibels; the digital or physical reference matters |
| [DDIM](glossary.md#ddim) | Denoising Diffusion Implicit Model |
| [ENU / NED](glossary.md#enu-ned) | East/North/Up; North/East/Down |
| [EXIF](glossary.md#exif) | Exchangeable Image File Format |
| [FC](glossary.md#fc) | Flight Controller |
| [FFC / NUC](glossary.md#ffc-nuc) | Flat-Field Correction / Non-Uniformity Correction |
| [FOV](glossary.md#footprint) | Field Of View |
| [FP32 / INT8](glossary.md#quantization) | 32-bit floating-point / 8-bit signed integer |
| [fps](glossary.md#units) | Frames per second |
| [GCS](glossary.md#gcs) | Ground Control Station |
| [GNSS / GPS](glossary.md#gnss) | Global Navigation Satellite System / Global Positioning System |
| [GP](glossary.md#gp) | Gaussian Process |
| [GSD](glossary.md#gsd) | Ground Sampling Distance |
| [Hz](glossary.md#units) | Hertz; cycles or updates per second |
| [INI / JSON](glossary.md#configuration-formats) | Settings format / JavaScript Object Notation |
| [IoU / NMS](glossary.md#iou-nms) | Intersection over Union / Non-Maximum Suppression |
| [IPC](glossary.md#ipc) | Inter-Process Communication |
| [JPEG / PNG / RGB](glossary.md#image-formats) | Image formats and red/green/blue colour channels |
| [LWIR](glossary.md#lwir) | Long-Wave Infrared |
| [MAD](glossary.md#hampel-filter) | Median Absolute Deviation |
| [MiB / RAM](glossary.md#ram) | Mebibytes / Random Access Memory |
| [MIMIC](glossary.md#mimic) | Map-tile camera simulation |
| [ms / us / ns](glossary.md#units) | Milliseconds / microseconds / nanoseconds |
| [MSL](glossary.md#msl) | Mean Sea Level |
| [NCHW / NHWC](glossary.md#tensor) | Image-tensor dimension orders |
| [NPS](glossary.md#nps) | New Paparazzi Simulator |
| [ONNX / ORT](glossary.md#onnx) | Open Neural Network Exchange / ONNX Runtime |
| [PANGEA](glossary.md#pangea) | Photo-Assisted Neighboring Geographic Edge Assembly |
| [PCM / RMS](glossary.md#pcm) | Pulse-Code Modulation; [Root Mean Square](glossary.md#rms) |
| [PTP / UVC](glossary.md#usb-and-csi) | Picture Transfer Protocol / USB Video Class |
| [ROI](glossary.md#roi) | Region Of Interest |
| [RPK](glossary.md#rpk) | The IMX500 model package |
| [SDK](glossary.md#sdk) | Software Development Kit |
| [SHA-256](glossary.md#sha256) | Secure Hash Algorithm 256-bit |
| [SSH](glossary.md#headless) | Secure Shell |
| [TX / RX / GND](glossary.md#uart) | Transmit / receive / ground |
| [UART](glossary.md#uart) | Universal Asynchronous Receiver/Transmitter |
| [USB](glossary.md#usb-and-csi) | Universal Serial Bus |
| [UTC](glossary.md#timestamp) | Coordinated Universal Time |
| [UTM](glossary.md#utm) | Universal Transverse Mercator |
| [WGS84](glossary.md#wgs84) | World Geodetic System 1984 |
| [XML](glossary.md#configuration-formats) | Extensible Markup Language |
| [XYZ](glossary.md#xyz-tiles) | Map-tile addresses, not height data |
