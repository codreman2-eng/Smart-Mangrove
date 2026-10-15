# Indirect ("side channel") waypoint-reached signals on the M600 / A3 via GS Pro / DJI Pilot waypoint actions

Context: ESP32 payload, receive-only on the A3 OSDK broadcast (~47 Hz), never activated, never commands the aircraft. Goal: learn "aircraft is at the drop waypoint".

Verdict summary (details and sources in each section):

| Mechanism | Verdict |
|---|---|
| A3 "PWM signal output to F-channel" waypoint action, read electrically by ESP32 | **Works with caveats.** Firmware support is documented. Which app exposes it, F-port access on the M600 and signal voltage are not verified. This is the only explicit, deliberate signal. |
| "Take photo" driving an F-port mapped as camera shutter, read by ESP32 | **Works with caveats (unverified end-to-end).** Firmware has shutter channel mapping. Reports from GS Pro / third-party ground stations conflict. |
| Hover/stay N s at the waypoint + position/speed gate (already implemented) | **Works.** Most robust with zero hardware change. It is a "stopped near coordinate" detector, not a true "mission reached" flag. |
| Rotate aircraft to a signature heading + hover, detected from the quaternion | **Works with caveats.** Good as a second confirmation factor. Heading tolerance and heading-mode interactions need a field test. |
| Gimbal pitch action read from the broadcast gimbal field | **Works with caveats only with a real DJI-supported gimbal (Ronin-MX / X3 / X5 / XT).** With no gimbal or a dummy: **does not** (unverified, but no evidence it reports anything). |
| Flight status / display mode showing "waypoint mission running" | **Does not (as documented).** OSDK DisplayMode has no waypoint value. An undocumented/reserved value may appear; needs a bench log. |

---

## 1. Which waypoint actions do GS Pro / DJI Pilot offer for the M600 / M600 Pro?

### Takeaway
GS Pro officially supports the M600/M600 Pro and the A3/N3. Per waypoint it offers up to 15 actions from this set: aircraft rotation, gimbal pitch, start/stop recording, photo capture and hover. Separately, A3 firmware 1.7.6.0 (2018-08-02) added a "PWM signal output to F-channel in Waypoint Actions", the only "custom/servo" style action found. DJI Pilot lists the M600 Pro, but its mission support on the M600 is ambiguous.

### Cited Findings
- GS Pro FAQ lists compatible platforms including "Matrice 600/600 Pro, A3, N3". Cameras: "Zenmuse X3, X5, X5R, X4S, X5S, Z3, Z30, XT2, and XT". Missions with more than 99 waypoints are auto-sectioned. Using the RC during a mission: "Yes, but this is not recommended as it may affect the mission." — [DJI GS Pro FAQ](https://www.dji.com/ground-station-pro/faq) (opened)
- "up to 99 Waypoints can be set with up to fifteen consecutive actions": "Aircraft Rotation, Stop and Start Recording, Gimbal Pitch, Photo Capture and Hover". Gimbal pitch range "between -90° (downwards) to 0° (forward)". — [Heliguy's Guide to the DJI GS Pro App](https://www.heliguy.com/blogs/posts/heliguys-guide-to-the-dji-gs-pro-app) (opened)
- DJI product page (search snippet only, not opened): "Each waypoint can have up to 15 consecutive actions, including aircraft rotation, gimbal pitch, start/stop recording, photo capture, and hover." — [DJI GS Pro product page](https://www.dji.com/au/ground-station-pro)
- Underlying Mobile SDK action semantics (GS Pro is built on MSDK). Stay: duration in ms, "[0, 32767] milliseconds". Rotate aircraft: actionParam in "[-180, 180] degrees", with direction set by the waypoint turnMode. (Search snippet of the MSDK doc; page not opened in full.) — [MSDK DJIWaypointActionType](https://developer.dji.com/iframe/mobile-sdk-doc/ios/Enums/DJIWaypointActionType.html)
- **A3 release notes, FW 1.7.6.0, 2018.08.02 (Assistant 2 for Autopilot v2.0.0):** "Added a PWM signal output to F-channel in Waypoint Actions." Also: "Fixed an issue where the aircraft will sometimes not fly to the next waypoint." and "Fixed an issue where the output of the F-channel is abnormal when used with the Matrice 600 Series Remote Controller Channel Expansion Kit." — [A3 Release Notes 20190109 (PDF)](https://dl.djicdn.com/downloads/a3/20190109/A3+release+notes+20190109.pdf) (downloaded, text extracted)
- Same PDF, FW 1.7.7.0, 2019.01.09: "Fixed issue: after setting the PWM signal to the F channel and restarting the flight controller, the F channel would output a minimum value before the preset value." — [A3 Release Notes (PDF)](https://dl.djicdn.com/downloads/a3/20190109/A3+release+notes+20190109.pdf)
- DJI Pilot: DJI's FlightHub download page lists "Matrice 600 Pro: Pilot PE v1.6.1" and "Matrice 600 Pro: Pilot v1.7.2" (search summary, not opened). A DJI forum staff reply reportedly said mission flight in Pilot was only for M200 series / Mavic 2 Enterprise (search summary; the forum page rendered empty when opened). — [DJI FlightHub downloads](https://www.dji.com/downloads/products/flighthub); [DJI forum thread 180019](https://forum.dji.com/thread-180019-1-1.html)

### Inferences
- The PWM-to-F-channel waypoint action is the A3 firmware capability. The ground-station UI that exposes it is most likely GS Pro, since that is DJI's waypoint app for the A3, but I could not confirm this from a GS Pro changelog or manual.
- Stay max is about 32.7 s per action (MSDK limit). Chaining several hover actions gives longer dwell (up to 15 actions per waypoint).
- DJI Pilot should not be relied on for M600 waypoint missions. GS Pro is the supported path.

### Gaps
- No official GS Pro user manual or GS Pro release-notes page was found or opened. I could not verify the exact GS Pro version or UI name for the F-channel PWM action, or whether it needs a specific A3/M600 firmware. The M600 Pro firmware bundle version that includes A3 FC 1.7.6.0+ is also unverified.
- I did not confirm whether the M600's integrated A3 exposes this action identically to a standalone A3.

## 2. Gimbal pitch as a side channel (broadcast gimbal field)

### Takeaway
The OSDK broadcast has a gimbal struct (roll/pitch/yaw in degrees plus per-axis limit flags), so a GS Pro "gimbal pitch" action would be visible, but only if the A3 has a DJI gimbal it recognises. On the M600 that list is Ronin-MX, Zenmuse X5 series, X3 and XT. There is no evidence that a dummy or third-party gimbal populates the field.

### Cited Findings
- Broadcast gimbal struct: "struct for data broadcast, return gimbal angle". Fields: roll, pitch, yaw (float32, "degree"), and pitchLimit/rollLimit/yawLimit ("1 - axis reached limit, 0 - no"). — [OSDK Telemetry::Gimbal struct](https://developer.dji.com/onboard-api-reference/structDJI_1_1OSDK_1_1Telemetry_1_1Gimbal.html) (opened)
- "M600 with A3 only supports Ronin-MX/Zenmuse X5 Series/Zenmuse X3/Zenmuse XT when operating gimbal control in angle." — [OSDK Gimbal class reference](https://developer.dji.com/onboard-api-reference/classDJI_1_1OSDK_1_1Gimbal.html) (opened)
- A3 manual: "The A3 and A3 Pro support the DJI Zenmuse Gimbal which can be enabled in DJI Assistant" (search summary of manual). — [A3 & A3 Pro User Manual (PDF)](https://dl.djicdn.com/downloads/a3/en/A3_and_A3_Pro_User_Manual_en_160520.pdf)
- A3 FW 1.7.0.5 (2017.05.11): "Added GPS information recording when taking stills with a third-party camera connected to a Ronin-MX." — [A3 Release Notes (PDF)](https://dl.djicdn.com/downloads/a3/20190109/A3+release+notes+20190109.pdf)

### Inferences
- With no CAN-connected DJI gimbal, the broadcast gimbal field is very likely static (zeros or stale). This is unverified. The ESP32 can check it in seconds by logging the field on the ground.
- A "cheap DJI-recognised gimbal" would be a used Zenmuse X3 (Inspire 1 camera) or XT, which appear on the supported list. Weight (~200+ g for X3), mounting and price are not verified here. A Ronin-MX is heavy and expensive. This adds mass and cost purely to carry a 1-axis signal, and it is worse than the F-port PWM option.
- Gimbal pitch is a good encoder if a gimbal is present: set an unusual pitch (e.g. -37°) only at drop waypoints, then return to normal after. The payload matches pitch within ±2° plus the position gate.

### Gaps
- No source confirms the A3 broadcast gimbal field's contents when no gimbal is attached. It also isn't confirmed whether GS Pro lets you add a gimbal-pitch action when no gimbal/camera is detected.
- No source on any "gimbal emulator" that the A3 accepts over CAN. I found nothing reliable.

## 3. Aircraft yaw as a side channel (rotate aircraft + hover)

### Takeaway
This works with caveats. GS Pro's "Aircraft Rotation" action sets an absolute yaw in [-180, 180]°, and the payload already gets attitude (quaternion) in the broadcast. A unique "signature heading" held during a hover at the drop waypoint is a cheap second factor, with no hardware change.

### Cited Findings
- GS Pro offers "Aircraft Rotation" as a waypoint action — [Heliguy guide](https://www.heliguy.com/blogs/posts/heliguys-guide-to-the-dji-gs-pro-app)
- MSDK rotate action: "[-180, 180] degrees", with rotation direction determined by the waypoint turnMode (search snippet) — [MSDK DJIWaypointActionType](https://developer.dji.com/iframe/mobile-sdk-doc/ios/Enums/DJIWaypointActionType.html)
- Broadcast includes quaternion and velocity (project background, already established on the ESP32).

### Inferences
- Recommended pattern at the drop waypoint: Hover 3 s, then Rotate to heading H_sig (chosen ≥30–45° away from the inbound/outbound leg courses and from other waypoints' rotations), then Hover N s, then the payload drops, then continue. The payload requires: within radius R of the planned coordinate, horizontal speed ≤ threshold, |yaw − H_sig| ≤ ~5° held for ≥1–2 s.
- Risks:
  - GS Pro heading mode. If "auto/along course" is set, the aircraft yaws along legs anyway, so pick H_sig off-course. If "custom/per-waypoint heading" mode is used, the yaw transition may already occur during the leg.
  - The pilot can yaw manually (GS Pro allows RC input during missions).
  - Compass/yaw error on the M600 is typically a few degrees, so keep the tolerance ±5–10°.
  - None of these risks has been field-validated.
- This is a confirmation factor, not an authoritative signal. Combined with the position/speed gate, a false positive requires the aircraft to be stopped at the right spot and pointing at the signature heading.

### Gaps
- No source quantifies GS Pro rotation accuracy or settling time on the M600. A field log is needed.

## 4. Hover/stay action + stop at a known coordinate

### Takeaway
This works and needs no change. A "Hover N s" action produces near-zero velocity at the waypoint. The launcher's existing gate (horizontal speed ≤ 2.0 m/s within radius of the planned coordinate) detects that. It cannot distinguish "mission reached waypoint" from "pilot stopped here", but within the radius that distinction rarely matters.

### Cited Findings
- Hover is a GS Pro waypoint action — [Heliguy guide](https://www.heliguy.com/blogs/posts/heliguys-guide-to-the-dji-gs-pro-app); [DJI GS Pro page (snippet)](https://www.dji.com/au/ground-station-pro)
- Stay range "[0, 32767] milliseconds" per action (MSDK, snippet) — [MSDK DJIWaypointActionType](https://developer.dji.com/iframe/mobile-sdk-doc/ios/Enums/DJIWaypointActionType.html)
- A3 FW 1.7.6.0 fixed "the aircraft will sometimes not fly to the next waypoint" (firmware version matters for mission reliability) — [A3 Release Notes (PDF)](https://dl.djicdn.com/downloads/a3/20190109/A3+release+notes+20190109.pdf)

### Inferences
- Tighten the gate with dwell. Require speed ≤ 0.5–1.0 m/s for ≥2 s rather than an instantaneous ≤ 2.0 m/s. A GS Pro hover produces a clean plateau near 0 m/s, while a pass-through at 2 m/s does not.
- Also require height within a band of the planned altitude.
- In GS Pro straight-line waypoint missions the aircraft decelerates at waypoints where actions execute. This suggests position + stop is sufficient for drops. (Inference from MSDK behaviour, not verified in GS Pro docs.)

### Gaps
- No GS Pro documentation found on deceleration behaviour at waypoints or curved-path vs action execution for the M600.

## 5. Camera trigger / PWM outputs on the A3 that a payload could read electrically

### Takeaway
This is the strongest true side channel. The A3 F1–F4 pins are PWM outputs and F5–F8 are PWM I/O. DJI Assistant 2 can map a channel to "Shutter", and A3 FW 1.7.6.0 added "PWM signal output to F-channel in Waypoint Actions". The ESP32 can read a servo-PWM pulse width on a GPIO, receive-only and without OSDK. Caveats: M600 physical access to the F ports, signal voltage, and which app and version exposes the action are not verified.

### Cited Findings
- A3 manual: "F5-F8 Pins Multifunction PWM I/O ports. F1-F4 Pins Multifunction PWM output ports." (search summary of manual) — [A3 & A3 Pro User Manual (PDF)](https://dl.djicdn.com/downloads/a3/en/A3_and_A3_Pro_User_Manual_en_160520.pdf)
- A3 FW 1.6.0.10 (2017.01.05): "Added channel mapping for the camera shutter in the map application, and GPS information recording when taking stills." Also "Added support for Lightbridge 2 Remote Controller Channel Expansion Kit". — [A3 Release Notes (PDF)](https://dl.djicdn.com/downloads/a3/20190109/A3+release+notes+20190109.pdf)
- A3 FW 1.7.6.0 (2018.08.02): "Added a PWM signal output to F-channel in Waypoint Actions." FW 1.7.7.0 fixed F-channel outputting a minimum value before the preset value after a restart. — [A3 Release Notes (PDF)](https://dl.djicdn.com/downloads/a3/20190109/A3+release+notes+20190109.pdf)
- Forum (search summaries; DJI forum pages rendered empty when opened, so unverified):
  - "connect up to DJI Assistant, and assign a 'Shutter' channel" for a single PWM output.
  - "you need the Channel Expansion kit to control the F ports" (for RC-driven control).
  - "the F port output is a standard PWM signal ... endpoints ... adjusted in the Assistant program".
  - The F ports reportedly do not supply V+.
  - Sources: [DJI forum: GSP and Shutter N3/A3](https://forum.dji.com/thread-118186-1-1.html); [DJI forum: A3 F ports](https://forum.dji.com/thread-95197-1-1.html)
- Conflicting report (UgCS, a third-party ground station): "If custom camera connected to N3/A3 autopilot using F1-F8 ports - we can't trigger it." This was attributed to DJI SDK limits. — [DIYDrones: DJI N3 + remote Shutter UGCS](https://diydrones.com/group/ugcs/forum/dji-n3-remote-shutter-ugcs) (search summary)
- Another post claims a third-party trigger needed Ronin-MX and RSS for automatic mission triggering (search summary) — [DJI forum thread 206392](https://forum.dji.com/thread-206392-1-1.html)

### Inferences
- Plan A, the explicit PWM waypoint action:
  - Assign one F port in Assistant 2 for waypoint PWM output.
  - In the mission, set the PWM high (e.g. 1900 µs) as an action at the drop waypoint (after a hover), and back low (1100 µs) after the dwell or at the next waypoint.
  - The ESP32 measures pulse width on a GPIO (RMT/pcnt/ledc capture) with common ground and a level shifter or divider if the signal is 5 V.
  - This is still passive and never commands the aircraft, so it is consistent with the project constraints. It needs one extra wire and an A3 FC firmware ≥1.7.6.0 (preferably 1.7.7.0).
- Plan B, the shutter mapping: map an F port as "Shutter" and use GS Pro "Take photo" at the drop waypoint. The ESP32 detects the shutter pulse. Reports conflict on whether this fires without a DJI camera, so bench-test it.
- UgCS's failure is consistent with a third-party MSDK app not having access. It does not rule out GS Pro or the newer firmware PWM action.
- Lightbridge 2: I found no evidence of a separate camera-trigger output on the LB2 air unit usable for this.

### Gaps
- Unverified: whether GS Pro (which version) exposes the F-channel PWM waypoint action on the M600 / M600 Pro, and its exact UI name.
- Unverified: M600 physical access. The A3 is integrated in the M600 frame; whether F1–F8 are reachable without the Channel Expansion Kit isn't known.
- Unverified: F-port logic level (3.3 V vs 5 V) and the idle value when the port is unassigned.
- I could not open the full A3 manual pages for Assistant 2 F-port function options (the PDF was not parsed). The DJI forum threads rendered empty.

## 6. Flight status / display mode: does the broadcast show "waypoint mission running"?

### Takeaway
Not per the documentation. The OSDK DisplayMode enum (the "Mode of the vehicle's state machine, as displayed on DJI Go") has no waypoint or mission value. MODE_HOTPOINT_MODE=9 exists, and MODE_NAVI_SDK_CTRL=17 is for OSDK API control. Values 13–14 and 18–32 are "reserved". The MSDK has a "GPSWaypoint" flight mode, so the FC does track a waypoint state internally. Whether it surfaces in the OSDK broadcast as an undocumented reserved value is unknown, and the cheapest test is to log the field during a GS Pro mission.

### Cited Findings
- DisplayMode values: MODE_MANUAL_CTRL 0, MODE_ATTITUDE 1, reserved 2–5, MODE_P_GPS 6, reserved 7–8, MODE_HOTPOINT_MODE 9, MODE_ASSISTED_TAKEOFF 10, MODE_AUTO_TAKEOFF 11, MODE_AUTO_LANDING 12, reserved 13–14, MODE_NAVI_GO_HOME 15, reserved 16, MODE_NAVI_SDK_CTRL 17, reserved 18–32, MODE_FORCE_AUTO_LANDING 33, reserved 34–39, MODE_SEARCH_MODE 40, MODE_ENGINE_START 41. — [OSDK VehicleStatus::DisplayMode](https://developer.dji.com/onboard-api-reference/namespaceDJI_1_1OSDK_1_1VehicleStatus_1_1DisplayMode.html) (opened)
- DisplayMode is described as "'Mode' of the vehicle's state machine, as displayed on DJI Go" — [OSDK VehicleStatus namespace](https://developer.dji.com/onboard-api-reference/namespaceDJI_1_1OSDK_1_1VehicleStatus.html) (opened)
- MSDK flight-mode enum includes "GPSWaypoint: GPS waypoint mode." and "GPSHotPoint: GPS hot point mode." (no numeric values on the page) — [MSDK DJIFlightControllerFlightMode](https://developer.dji.com/iframe/mobile-sdk-doc/android/reference/dji/common/flightcontroller/DJIFlightControllerFlightMode.html) (opened)
- Broadcast mode is the older telemetry path; the M100 and certain M600 versions support only broadcast (search summary; the article is login-restricted) — [DJI SDK forum article 4406225846425](https://sdk-forum.dji.net/hc/en-us/articles/4406225846425)

### Inferences
- HOTPOINT has a named value, and the MSDK enum lists Waypoint next to HotPoint. So a GS Pro waypoint mission plausibly reports a reserved value (13 or 14 are candidates) rather than 6. This is speculation. If it holds on the bench, `mode == X` plus the position/hover gate gives a strong "mission is executing" qualifier, but still not "at waypoint k".
- The flight status field (stopped / on ground / in air) gives no mission information.
- The broadcast "SDK control mode / control device" field relates to OSDK authority, not MSDK missions. Expect it to stay unchanged during GS Pro missions (unverified).

### Gaps
- No source states which DisplayMode value the A3 reports during a GS Pro (MSDK) waypoint mission. An empirical ESP32 log (mode byte vs time during a short 3-waypoint test mission) is required and cheap.
