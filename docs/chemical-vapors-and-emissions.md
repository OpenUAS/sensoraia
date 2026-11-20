# Chemical Vapors and Emissions

There is more to a landscape than a camera can show. With the right onboard
sensors, a drone can sample carbon dioxide in the air, look for methane leaks,
or survey radiation emitted from the ground. It is another way to explore a
place: not just what it looks like, but what is happening around it. Pair the
readings with the aircraft's position and time, and an invisible change becomes
something you can follow, compare and investigate, whether you are checking a
biogas installation, studying air quality or looking for an unusual emission.

## Ten Things A Drone Can Detect

Each example needs a suitable sensor; no single instrument detects everything
in this list.

1. **Carbon dioxide (CO2).** Measure changes around combustion sources,
   industrial processes or natural releases with an infrared gas sensor.
2. **Methane (CH4).** Look for leaks from gas infrastructure, landfills or
   biogas facilities with a methane-sensitive instrument.
3. **Carbon monoxide (CO).** Detect this combustion gas in smoke or exhaust
   using an appropriate gas sensor.
4. **Ammonia (NH3).** Check for releases around livestock buildings,
   fertilizer handling or industrial refrigeration.
5. **Hydrogen sulfide (H2S).** Detect a gas associated with wastewater,
   decomposing organic material and some industrial operations.
6. **Sulfur dioxide (SO2).** Investigate emissions from sulfur-containing
   fuels, industrial processes or volcanic activity.
7. **Nitrogen dioxide (NO2).** Study a combustion-related air pollutant
   found in traffic and industrial exhaust.
8. **Volatile organic compounds (VOCs).** Screen for groups of vapors from
   fuels, paints and solvents. Identifying an individual compound needs a
   more selective instrument or analysis.
9. **Airborne particles, such as dust and smoke.** Measure particle levels
   with an optical particle sensor; particles are not gases or vapors.
10. **Gamma radiation.** Survey radiation levels with a dedicated detector.
    A suitable gamma spectrometer can also help identify radioactive materials.

## Different Signals, Different Sensors

For many gas measurements, a small pump draws air past a sensor on the aircraft.
Other instruments use light to detect particular gases along a viewing path.
Radiation sensing is different: the detector measures radiation reaching it,
not the smell or chemical makeup of an air sample. A normal camera or thermal
imager does not replace these specialist instruments.

Wind moves and dilutes a gas plume, and propeller airflow can change the air
reaching an inlet. Sensor placement, response time and calibration therefore
matter just as much as where the drone flies. For radiation surveys, distance,
shielding, background levels and measurement time affect what can be detected.
A higher reading is a useful clue, not automatically the source location or
the amount being released.

## Bringing A New Sensor To Sensoraia

Gas, particle and radiation payloads need their own compatible connection and
Sensoraia integration; these are not ready-to-use sensor backends included
with the camera and microphone modules. The aim is familiar: keep each reading
with its units, measurement time and flight context, so it can become a useful
part of an airborne sensing workflow.

Explore [Sensoraia's current sensors](usher-camera-pipeline.md#choose-a-camera)
or see how [the autopilot link](autopilot-integration.md) connects airborne
observations to the rest of a mission.

## Explore Real-World Examples

At the moment there is not yet a sensor module in this class but if you need it now
then add code andd behaviour to make it a reallity. We look froward to integrate
a new class of sensor create by you into Sensoraia. Please make a pull request 
if you feel you are ready to share it with the world.