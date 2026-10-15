# DJI OSDK (3.x / 4.x): waypoint-mission status on M600 / A3 / A3 Pro without control authority

Scope note: research done 2026-10-14 against the dji-sdk/Onboard-SDK `3.9` branch source (raw GitHub), the DJI developer site (OSDK docs, protocol docs), and GitHub issues. The DJI developer forum thread that was opened returned an empty page. Quotes are exact where marked. "UNVERIFIED" marks claims I could not confirm from a primary source.

## Q1. How do OSDK 3.x waypoint push data / mission-status callbacks work, and do they need activation only or control authority?

### Takeaway
OSDK 3.x has two waypoint push streams on CMD set 0x02: **0x02 0x03 "Ground Station State" (mission status: mission_type, target_waypoint, current_status, error_notification)** and **0x02 0x04 "Waypoint Event" (upload finish / mission finish / waypoint reached with waypoint_index)**. They reach user code through `setWaypointCallback` and `setWaypointEventCallback`. The host library's dispatch code checks neither activation nor control authority before it calls these handlers. Control authority is needed to *upload/start* a mission, not to *receive* the pushes. Whether the flight controller transmits 0x02 0x03 / 0x02 0x04 to a never-activated device is not documented anywhere I found (UNVERIFIED).

### Cited Findings
- The protocol defines the push set: "0x02 0x00: Flight Data" (broadcast, content configured in DJI Assistant), "0x02 0x01: Lost of Flight Control", "0x02 0x02: Data transparent transmission from Mobile SDK", "**0x02 0x03: Ground Station State**", "**0x02 0x04: Waypoint Event**". The page has no per-command "activation required / authorization" column. — [OSDK OPEN Protocol](https://developer.dji.com/onboard-sdk/documentation/protocol-doc/open-protocol.html)
- The Ground Station protocol calls these "not API but broadcast data protocol". It lists 0x02 0x03 as "Current Mission Status Push Information" and 0x02 0x04 as "Mission Event Push Information". Waypoint status struct: `mission_type, target_waypoint, current_status, error_notification, reserved`. "There are four kinds of mission status with the same struct size", told apart by `mission_type` (ATTI/IOC default, waypoint, hotpoint, follow-me). Waypoint events: `incident_type` ∈ {`NAVI_UPLOAD_FINISH`, `NAVI_MISSION_FINISH`, `NAVI_MISSION_WP_REACH_POINT`}. The reach-point struct has `incident_type, waypoint_index, current_status, reserved_1, reserved_2`. Upload-finish has `is_mission_valid, estimated_run_time`. — [Ground Station Protocol](https://developer.dji.com/onboard-sdk/documentation/protocol-doc/ground-station-protocol.html)
- That page's only stated prerequisite is in DJI Assistant: "Developers should select the `Ground Station Status` checkbox in DJI Assistant." It says nothing about activation or control authority. (This doc is from the M100/legacy era; I did not confirm that the checkbox exists in DJI Assistant 2 for A3/M600.) — [Ground Station Protocol](https://developer.dji.com/onboard-sdk/documentation/protocol-doc/ground-station-protocol.html)
- API reference struct `DJI::OSDK::ACK::WayPointReachedData` (incident type, waypoint index, current status, "index of current waypt mission"). — [WayPointReachedData reference](https://developer.dji.com/onboard-api-reference/structDJI_1_1OSDK_1_1ACK_1_1WayPointReachedData.html)
- `dji_waypoint.hpp` (3.9) declares the members `VehicleCallBackHandler wayPointEventCallback;` and `VehicleCallBackHandler wayPointCallback;`, plus `void setWaypointEventCallback(VehicleCallBack callback, UserData userData);` (doc comment: "Set waypoint push data callback") and `void setWaypointCallback(VehicleCallBack callback, UserData userData);` ("Set waypoint callback"). — [dji_waypoint.hpp @3.9](https://raw.githubusercontent.com/dji-sdk/Onboard-SDK/3.9/osdk-core/api/inc/dji_waypoint.hpp)
- `dji_vehicle.cpp` (3.9) push dispatch: `processReceivedData` routes non-ACK frames to `PushDataHandler(...)`. Branch `else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Broadcast::mission, ...` → `if (missionManager)` → `switch (pushDataEntry->recvData.missionACK)` → `case MISSION_WAYPOINT:` → `if (missionManager->wpMission->wayPointCallback.callback)`. `case MISSION_IOC:` only logs ("//! @todo compare IOC with other mission modes comprehensively"). Branch `else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Broadcast::waypoint, ...` → `missionManager->wpMission->wayPointEventCallback.callback(`. **No activation-status or control-authority check is made before dispatch.** `getActivationStatus()` exists but is not used there. Caveats: the dispatch needs a `missionManager` / `wpMission` object, i.e. `missionManager->init(WAYPOINT, ...)` must have run, and the `waypoint` branch dereferences `missionManager->wpMission` without null-checking `missionManager`. — [dji_vehicle.cpp @3.9](https://raw.githubusercontent.com/dji-sdk/Onboard-SDK/3.9/osdk-core/api/src/dji_vehicle.cpp)
- Control authority *is* required to command missions. In the protocol ACK tables, waypoint upload and other commands return codes meaning control must be obtained first, e.g. Arm/Disarm ACK 0x0001 "Please obtain control at first". — [OSDK OPEN Protocol](https://developer.dji.com/onboard-sdk/documentation/protocol-doc/open-protocol.html)
- The 3.9 Linux `mission_sample.cpp` runs `missionManager->init(DJI_MISSION_TYPE::WAYPOINT, ...)` → upload → `wpMission->start(...)`. That file contains no `obtainCtrlAuthority` call. (I did not open the sample's `main.cpp`, where I believe authority is obtained; UNVERIFIED.) — [mission_sample.cpp @3.9](https://raw.githubusercontent.com/dji-sdk/Onboard-SDK/3.9/sample/platform/linux/missions/mission_sample.cpp)
- DJI staff (issue #93) explain the difference: `mission_callback` path → `setWayPointCallback` (status, 0x02 0x03), and `waypoint_event_callback` path → `setWayPointEventCallback` (events, 0x02 0x04). Each falls back to a printf if unset. — [Issue #93](https://github.com/dji-sdk/Onboard-SDK/issues/93)
- Known reliability problems. In coordinated-trace mode (`TRACE_COORDINATED`), only the first and last waypoint events arrive ([Issue #219](https://github.com/dji-sdk/Onboard-SDK/issues/219)). On M210 RTK with OSDK-ROS 3.7, the waypoint event callback "is never called neither on the simulator nor in a real flight" ([Onboard-SDK-ROS #244](https://github.com/dji-sdk/Onboard-SDK-ROS/issues/244); that is M210, not M600).

### Inferences
- Because the library does not gate dispatch on activation or authority, the only thing that could stop an unactivated, receive-only listener from seeing these pushes is the flight controller itself (whether it emits 0x02 0x03 / 0x02 0x04 to a non-activated API port). The ESP32 already parses OSDK frames on the API UART, so the cheapest test is empirical: log every frame's CMD set/ID while a GS Pro mission flies (in the DJI Assistant 2 simulator) and look for set 0x02, ID 0x03/0x04 frames.
- The OSDK C++ library cannot be used as-is for this, because it calls `missionManager->init()`, and an init that sends commands requires TX. A custom parser on the ESP32 avoids that.

### Gaps
- No DJI doc states whether 0x02 0x03 / 0x02 0x04 are emitted before activation, or whether they are emitted at all for missions not uploaded by the OSDK.
- I could not confirm that DJI Assistant 2 for A3/M600 has a "Ground Station Status" checkbox. The doc that mentions it is legacy/M100.

## Q2. Do the push callbacks report missions uploaded and started by other apps (GS Pro, DJI Pilot, MSDK apps)?

### Takeaway
DJI never confirmed it, and its official answer for OSDK-ROS was negative for reading mission info set by DJI or third-party apps. DJI said it had "not tested" subscribing to mission information while a third-party app generated the mission. This is the M600 Pro case. Treat it as unsupported and test it empirically.

### Cited Findings
- Issue #303 (M600 Pro, FW 1.0.1.67, OSDK 3.8.1). DJI support (dji-dev, Zendesk #28919, 2019-10-17): "we have not tested using DJI OSDK-ROS to subscribe to mission information while using a third party APP to generate a mission." Official follow-up (2020-07-10): "**OSDK-ROS does not support obtaining waypoint task information set by DJI APP/ third-party APP.**" The reporter (erenaud3) found that `missionWpGetInfoCallback` "can only download waypoints from drone if and only if we already uploaded some". He proposed a workaround that builds an empty mission to download settings from the drone. A later user said this was useful and adapted it to stop third-party-started missions. That implies the mission objects can be reached, but those operations *send* commands. — [Onboard-SDK-ROS #303](https://github.com/dji-sdk/Onboard-SDK-ROS/issues/303)
- Issue #498 (M100, OSDK 3.9). For phase-of-mission tracking DJI said: "For waypoint tasks created by OSDK, you can also register waypoint callbacks to get the current task status in real-time". The answer is scoped to OSDK-created tasks. — [Onboard-SDK #498](https://github.com/dji-sdk/Onboard-SDK/issues/498)
- The ground-station protocol's mission-status push has a default ATTI/IOC variant with `last_mission_type` and `is_broken`. This suggests the FC reports its navigation state regardless of who started it, but the docs don't say so. — [Ground Station Protocol](https://developer.dji.com/onboard-sdk/documentation/protocol-doc/ground-station-protocol.html)
- A contributor (lanyusea) on rc_lost_action: "mission tasks are not exclusive to the Onboard SDK". He meant the FC's mission engine is shared between MSDK and OSDK. — [Issue #416](https://github.com/dji-sdk/Onboard-SDK/issues/416)

### Inferences
- GS Pro / DJI Pilot missions on A3/M600 run on the same FC waypoint engine as OSDK missions. Pushes for them may be emitted (the push is FC-side), but DJI explicitly does not support or guarantee it. Only a test settles it.

### Gaps
- I found no report, positive or negative, of someone actually seeing 0x02 0x03 / 0x02 0x04 frames on the API port during a GS Pro mission on A3/M600. The DJI forum thread fetch returned nothing.

## Q3. Does activation alone unlock any mission-related subscription topic? Does DISPLAYMODE reveal waypoint mode? Is subscription available on M600/A3, and does it need activation?

### Takeaway
No. The OSDK 3.9 subscription topic list has **no mission-state or waypoint-index topic**. `TOPIC_STATUS_DISPLAYMODE` (flight mode) has no named waypoint/GS mode in the 3.9 enum: it names hotpoint (9) and SDK control (17) but leaves 13-14, 16 and others "reserved". Subscription (0x03 set) is itself a command/response protocol, so it needs TX. The 3.9 sample skips it on "legacy M600".

### Cited Findings
- Full `TopicName` list in 3.9 `dji_telemetry.hpp`: QUATERNION, ACCELERATION_GROUND/BODY/RAW, VELOCITY, ANGULAR_RATE_FUSIONED/RAW, ALTITUDE_FUSIONED/BAROMETER, HEIGHT_HOMEPOINT, HEIGHT_FUSION, GPS_FUSED/DATE/TIME/POSITION/VELOCITY/DETAILS, RTK_POSITION/VELOCITY/YAW/POSITION_INFO/YAW_INFO, COMPASS, RC, GIMBAL_ANGLES/STATUS, STATUS_FLIGHT, STATUS_DISPLAYMODE, STATUS_LANDINGGEAR, STATUS_MOTOR_START_ERROR, BATTERY_INFO, CONTROL_DEVICE, HARD_SYNC, GPS_SIGNAL_LEVEL, GPS_CONTROL_LEVEL, RC_FULL_RAW_DATA, RC_WITH_FLAG_DATA, ESC_DATA, RTK_CONNECT_STATUS, GIMBAL_CONTROL_MODE, FLIGHT_ANOMALY, POSITION_VO, AVOID_DATA, HOME_POINT_SET_STATUS, HOME_POINT_INFO. **No mission/waypoint topic.** RTK struct note: "Available on A3/M600, need to enable it separately on DJI Assistant 2." `SDKInfo` (TOPIC_CONTROL_DEVICE and broadcast): `controlMode`, `deviceStatus` "0->rc 1->app 2->serial", `flightStatus` "1->opensd 0->close", `vrcStatus`. — [dji_telemetry.hpp @3.9](https://raw.githubusercontent.com/dji-sdk/Onboard-SDK/3.9/osdk-core/api/inc/dji_telemetry.hpp)
- DisplayMode enum in 3.9 `dji_status.hpp`: `MODE_MANUAL_CTRL=0, MODE_ATTITUDE=1, MODE_P_GPS=6, MODE_HOTPOINT_MODE=9, MODE_ASSISTED_TAKEOFF=10, MODE_AUTO_TAKEOFF=11, MODE_AUTO_LANDING=12, MODE_NAVI_GO_HOME=15, MODE_NAVI_SDK_CTRL=17` ("the aircraft is controled by SDK API"), `MODE_FORCE_AUTO_LANDING=33, MODE_SEARCH_MODE=40, MODE_ENGINE_START=41`. Values 2-5, 7-8, 13-14, 16, 18-32, 34-39, 42-43 are reserved/unnamed. **No waypoint value is named.** — [dji_status.hpp @3.9](https://raw.githubusercontent.com/dji-sdk/Onboard-SDK/3.9/osdk-core/api/inc/dji_status.hpp)
- The 3.9 mission sample sets up subscription "only when the vehicle is not M100 or legacy M600" (`subscribe->verify`, `startPackage`). Older M600 firmware falls back to broadcast. — [mission_sample.cpp @3.9](https://raw.githubusercontent.com/dji-sdk/Onboard-SDK/3.9/sample/platform/linux/missions/mission_sample.cpp)
- The broadcast (0x02 0x00) content and rate ([0,100 Hz]) are configured in DJI Assistant 2. — [OSDK FAQ](https://developer.dji.com/onboard-sdk/documentation/appendix/FAQ.html); [OPEN Protocol](https://developer.dji.com/onboard-sdk/documentation/protocol-doc/open-protocol.html)

### Inferences
- Display mode is already in the broadcast the ESP32 receives (flight status/mode fields). While GS Pro flies a mission, the A3 most likely reports a value from the unnamed/reserved range. From DJI MSDK flight-mode naming (GPS_WAYPOINT), I *believe* it is 14, but this is UNVERIFIED for OSDK. Logging display mode during a simulated GS Pro mission would give a cheap "mission active" flag without any TX. It would not give a waypoint index.
- Subscription adds nothing mission-specific and needs TX, so it offers no benefit over broadcast for this project.

### Gaps
- I found no official mapping of the reserved DisplayMode values to "waypoint mission" for A3/M600.
- No source states explicitly whether subscription requires activation. Subscription is a request/ACK protocol, so it at least needs TX.

## Q4. Does activation require TX and internet/App ID+key? Does activation by itself affect flight behaviour or failsafes?

### Takeaway
Yes on all requirements. Activation is command 0x00 0x01, sent by the onboard device (TX required). It carries a DJI-developer `app_id`, and the key is used for encryption. It needs "Enable API Control" in DJI Assistant 2 and an internet-connected DJI GO / Assistant 2 on first use. Activation alone is documented only as a prerequisite. No source says it changes flight behaviour. The behavioural changes (SDK mode, SDK failsafe) come from control authority plus the DJI Assistant 2 SDK settings.

### Cited Findings
- Activation command (0x00 0x01): CMD Val is a 4-byte `app_id`, plus a fixed value (A3: `0x03016400`) and a fixed 32-byte string. ACK codes include "New app ID; connect the DJI GO app", "No response from the DJI GO app", "No internet connection from the DJI GO app", "Server rejected the request", "Authorization level insufficient". Get Protocol Version (0x00 0x00) returns "Autopilot is activated" (0x0000) / "Autopilot is NOT activated" (0xFF01). — [OPEN Protocol](https://developer.dji.com/onboard-sdk/documentation/protocol-doc/open-protocol.html)
- The FAQ says "DJI GO needs to be running on the mobile device" and that internet is needed on the first run only, with the result cached. "Go to the SDK page on Assistant 2 and check the box marked Enable API Control." "Currently, you do need a LightBridge 2 RC for first use (specifically, activation)" (A3). The FAQ notes it was written for OSDK 3.2. — [OSDK FAQ](https://developer.dji.com/onboard-sdk/documentation/appendix/FAQ.html)
- The safety page says: "Please set the APP ID and APP KEY for Application activation", and to activate while connected to DJI Assistant 2 and online. — [OSDK Safety](https://developer.dji.com/onboard-sdk/documentation/quickstart/safety.html)
- Library side (3.9 `Vehicle::activate`): on success, `protocolLayer->setKey(accountData.encKey); setActivationStatus(true);` and, if no gimbal object yet, `initGimbal()`. The error text says "Double-check your app_id and app_key in UserConfig.txt." — [dji_vehicle.cpp @3.9](https://raw.githubusercontent.com/dji-sdk/Onboard-SDK/3.9/osdk-core/api/src/dji_vehicle.cpp)
- The control-authority guide lists three separate prerequisites for OSDK control: mode switch in P (A3, N3, M600, M600 Pro), "the onboard application must have activated", and "must request and receive control authority". Without these, the onboard computer can still receive state information. — [Control Authority guide](https://developer.dji.com/onboard-sdk/documentation/guides/component-guide-control-authority.html)
- Re-activation is needed if the device was previously activated with another app_id/app_key. — [Issue #439 search snippet](https://github.com/dji-sdk/Onboard-SDK/issues/439) (snippet only, not opened)

### Inferences
- Activation would break the project's receive-only, never-TX design: a TX wire, a DJI developer account/App ID+key, and an internet-connected first run. It also swaps in an encryption key, which may change framing of later traffic if encryption is enabled. Activation alone has no documented flight effect, but nothing mission-related is documented as unlocked by activation either.

### Gaps
- No primary source explicitly says "activation without authority has no effect on failsafe". This is inferred from the docs tying failsafe/SDK mode to authority and Assistant settings.

## Q5. OSDK 4.x: is the M600 supported? What is the last OSDK supporting M600/A3/N3? Waypoint V2 on M600?

### Takeaway
**OSDK 3.9 (released 24 Sep 2019) is the last line that lists M600/M600 Pro, A3/A3 Pro and N3.** OSDK 4.x lists only M300 RTK and M210 (RTK) V2. Waypoint V2 is a 4.x / M300 feature. In 3.9 its push handling is compiled only under `#ifdef WAYPT2_CORE`. No source claims Waypoint V2 on M600.

### Cited Findings
- 3.9 README compatibility table: "M600/M600 Pro | 1.0.1.67 | 3.2.41.14 | OSDK 3.9"; "A3/A3 Pro | 1.7.7.0 | 3.3.8.47 | OSDK 3.9"; "N3 | 1.7.7.0 | 3.3.8.47 | OSDK 3.9"; and "OSDK 3.9 was released on 24 September 2019." — [README @3.9](https://raw.githubusercontent.com/dji-sdk/Onboard-SDK/3.9/README.md)
- Master README (4.1.0, "released on 2 February 2021"): the firmware table lists only M300 RTK, M210 RTK V2 and M210 V2. — [README @master](https://raw.githubusercontent.com/dji-sdk/Onboard-SDK/master/README.md)
- A forum user notes that since 4.0 the docs mention only M300 and 2xx, not A3, N3 or M100. — [DJI forum thread 217814](https://forum.dji.com/thread-217814-1-1.html) (from the search snippet; the page fetch returned empty, so UNVERIFIED as a direct quote)
- 3.9 `dji_vehicle.cpp`: Waypoint V2 state push (`OpenProtocolCMD::CMDSet::Mission::waypointGetStatePushDataV2` → `updateV2PushData`) only exists `#ifdef WAYPT2_CORE`. — [dji_vehicle.cpp @3.9](https://raw.githubusercontent.com/dji-sdk/Onboard-SDK/3.9/osdk-core/api/src/dji_vehicle.cpp)
- The hardware intro page marks Waypoint "Yes" for M100/210/210 RTK/600/600 Pro without saying which version. The only V2 mention is a sample "Waypoint Mission V2 (beta)". — [OSDK hardware intro](https://developer.dji.com/onboard-sdk/documentation/introduction/osdk-hardware-introduction.html)

### Inferences
- The project is limited to OSDK 3.x / waypoint V1 protocol semantics (0x02 0x03 / 0x02 0x04) on the A3.

### Gaps
- I found no official "end of support" statement for M600/A3/N3. The conclusion comes from the compatibility tables.

## Q6. What happens with the SDK failsafe / Return-Home when control authority is held and the onboard link is lost?

### Takeaway
The docs describe **RC-link loss** behaviour, not onboard-serial-link loss. By default, RC loss → RTL (failsafe RTH after >3 s). If "Enable SDK Failsafe Action" is checked in DJI Assistant 2, the SDK failsafe action (e.g., Hover) applies instead and lets OSDK keep executing. The protocol has a 0x02 0x01 "Lost of Flight Control" push, sent when control is lost from the onboard system. I found **no DJI doc that states what the A3 does when the onboard serial/heartbeat is lost while authority is held** (UNVERIFIED). Users asked DJI for exactly that heartbeat behaviour, and DJI did not answer.

### Cited Findings
- "By default, when the remote controller loses connection to the vehicle, the vehicle executes a return-to-home-and-land (RTL) maneuver. However, there is now an option to override this behavior by allowing the OSDK to continue executing its mission. To do so, users will have to check the Enable SDK Failsafe Action box and set the SDK Failsafe Action to Hover." Priority order: RC > Mobile device > Onboard computer, and RC "can regain control from the mobile device or onboard computer at any time". M600/M600 Pro: "all flight mode switch positions are P mode always". The page does not cover onboard-link loss. — [Control Authority guide](https://developer.dji.com/onboard-sdk/documentation/guides/component-guide-control-authority.html)
- "Failsafe RTH will be automatically activated if the remote controller signal is lost for more than three seconds", provided "the Home Point was successfully recorded and the compass is functioning normally". Failsafe behaviour (RTH / hover / land) is configurable in DJI Assistant 2. — [Flight Controller guide](https://developer.dji.com/onboard-sdk/documentation/guides/component-guide-flight-control.html)
- Issue #278 (A3/M600). A user wanted "the aircraft would RTH if heartbeat signal is abnormal because of SDK crash or other reason". DJI-side reply: "This is already available, you can enable it in DJI Assistant 2's SDK page" (Enable SDK Failsafe → Hover). The heartbeat/onboard-loss request was never answered, and the issue was closed in 2019. — [Issue #278](https://github.com/dji-sdk/Onboard-SDK/issues/278)
- Issue #335: after RC loss and recovery, the drone can remain stuck in OSDK failsafe and reject control even after authority is released. — [Issue #335 (search snippet)](https://github.com/dji-sdk/Onboard-SDK/issues/335) (not opened)
- Issue #416: `rc_lost_action` (mission param, 1 = continue mission) vs the SDK failsafe. Contributor: "the `rc_lost_action` is only provided by mission tasks". The two "should be an OR action" but this needed checking. Never resolved. — [Issue #416](https://github.com/dji-sdk/Onboard-SDK/issues/416)
- Push 0x02 0x01 "Lost of Flight Control": sent by the autopilot "when flight control is lost from the onboard system", CMD Val fixed 0x04. — [OPEN Protocol](https://developer.dji.com/onboard-sdk/documentation/protocol-doc/open-protocol.html)

### Inferences
- Every SDK-failsafe mechanism found is tied to OSDK being the controlling device (SDK mode / authority) plus DJI Assistant 2 SDK settings. A device that never activates and never requests authority stays out of that path. That supports the project's receive-only stance, but no doc explicitly confirms it.

### Gaps
- There is no primary documentation of A3/M600 behaviour on onboard serial or heartbeat loss while authority is held.
- I could not open the DJI Assistant 2 SDK-page documentation for A3/M600, so I have no exact option list or default values.
