# DJI Mobile SDK (MSDK 4.x) phone app as a waypoint-progress observer and relay for the M600 / A3

Overall verdict: **works with heavy caveats. In practice the custom app would have to *be* the mission app.** MSDK 4.x on the M600 exposes exactly the fields wanted (`targetWaypointIndex`, `isWaypointReached`, `executeState`). But the custom app cannot run alongside GS Pro or DJI Pilot on the same RC link. Seeing a GS Pro mission is documented only as a "reconnect" recovery path, never tested on M600 for a foreign-app mission. The relay to the ESP32 has two paths: the ESP32's own Wi-Fi (range-limited) or MSDK `sendDataToOnboardSDKDevice` to the A3 API port (likely needs OSDK activation, not control authority; unverified).

## 1. Does WaypointMissionOperator (v1) report progress for missions uploaded or started by another app (GS Pro / Pilot)?

### Takeaway
The listener reports exactly the data needed. DJI documents that if a waypoint mission is "already executing" when the SDK (re)connects, the operator auto-loads a summary of it and keeps tracking execution. That is the only documented route to a mission the app did not upload. No source confirms it works for a GS Pro-started mission on an M600, so treat it as plausible but untested.

### Cited Findings
- iOS: "The waypoint operator is the only object that controls, runs and monitors Waypoint Missions." The class is marked "Will no longer be maintained", and M300 users are told to use `DJIWaypointV2MissionOperator`. Execution listener is `addListenerToExecutionEvent:withQueue:andBlock:` with block type `DJIWaypointMissionOperatorExecutionEventBlock`. — [DJIWaypointMissionOperator (iOS)](https://developer.dji.com/api-reference/ios-api/Components/Missions/DJIWaypointMissionOperator.html)
- `loadedMission` / `getLoadedMission`: "If the aircraft is already executing a waypoint mission when SDK is re-connected, the operator will download part of the mission's information from the aircraft and load it automatically." The loaded mission "will only contain the summary of the executing mission but information for each waypoint is absent". `downloadMission` fetches per-waypoint data and can only be called in `EXECUTING` or `EXECUTION_PAUSED`. `getLoadedMission` "will be reset to null when the execution of the loadedMission is stopped, finished or interrupted." — [WaypointMissionOperator (Android)](https://developer.dji.com/api-reference/android-api/Components/Missions/DJIWaypointMissionOperator.html); same text on [iOS page](https://developer.dji.com/api-reference/ios-api/Components/Missions/DJIWaypointMissionOperator.html)
- Only `getPreviousInterruption` is marked "Supported only by Phantom 4 RTK". The reconnect auto-load is not restricted to a product on the page. — [DJIWaypointMissionOperator (iOS)](https://developer.dji.com/api-reference/ios-api/Components/Missions/DJIWaypointMissionOperator.html)
- `DJIWaypointMissionState` values: `NotSupported`, `ReadyToUpload`, `Uploading`, `ReadyToExecute`, `Executing`, `ExecutionPaused`, `Disconnected`, `Recovering`, `Unknown`. — [DJIWaypointMissionOperator (iOS)](https://developer.dji.com/api-reference/ios-api/Components/Missions/DJIWaypointMissionOperator.html)
- `DJIWaypointMissionExecutionEvent` properties: `progress` (`DJIWaypointExecutionProgress`, "nil if there is an error"), `previousState`, `currentState`, `error`. — [DJIWaypointMissionExecutionEvent (iOS)](https://developer.dji.com/api-reference/ios-api/Components/Missions/DJIWaypointMissionExecutionEvent.html)
- `WaypointExecutionProgress` (Android):
  - `targetWaypointIndex`: "Index of the waypoint for the next mission to which the aircraft will proceed."
  - `isWaypointReached`: "YES when the aircraft reaches a waypoint. After the waypoint actions and heading change is complete, the targetWaypointIndex will increment and this property will become NO."
  - `executeState`: "Current execution state of the aircraft" (`WaypointMissionExecuteState`).
  - Source: [DJIWaypointExecutionProgress (Android)](https://developer.dji.com/api-reference/android-api/Components/Missions/DJIWaypointMission_DJIWaypointExecutionProgress.html); [iOS equivalent](https://developer.dji.com/api-reference/ios-api/Components/Missions/DJIWaypointMission_DJIWaypointExecutionProgress.html)
- On Android the listener is `WaypointMissionOperatorListener.onExecutionUpdate(WaypointMissionExecutionEvent)`, with `onExecutionStart` / `onExecutionFinish`. This comes from the search summary of DJI pages and the DJI forum article "How to listen the waypointMission execution status". The forum article itself is behind a login and was not opened. — [Search hit: DJI SDK forum 4404658338969](https://sdk-forum.dji.net/hc/en-us/articles/4404658338969-How-to-listen-the-waypointMission-execution-status) (UNVERIFIED body)
- GS Pro missions: I found no forum thread or GitHub issue confirming or denying that a GS Pro-uploaded mission shows up in a third-party MSDK app's `WaypointMissionOperator`.

### Inferences
- The MSDK operator is a client of the flight controller's waypoint engine, which is mission-source agnostic: a GS Pro mission is still a v1 waypoint mission in the A3. So the "already executing when SDK is re-connected" path *should* fire for a GS Pro mission once the custom app takes the link. The app would then get `onExecutionUpdate` with `targetWaypointIndex`. This is an inference that must be bench-tested.
- GS Pro "3D Map / Photogrammetry" missions are also uploaded as waypoint missions (believed, not verified). GS Pro "Virtual Fence" and tap-and-go modes may not be.
- Practically this means: GS Pro uploads and starts, then the pilot switches the USB link or app to the custom app, which recovers and monitors. GS Pro loses its live view and control while the custom app holds the link.

### Gaps
- No primary evidence for a foreign-app mission on M600/A3. Login-gated DJI forum posts could not be read.
- Whether the A3 firmware supports the "download executing mission" feature (it may be Phantom/Inspire-centric) is unverified.
- DJI Pilot (the original, not Pilot 2) on M600: whether it uses the same v1 waypoint engine was not checked.

## 2. Can GS Pro / DJI Pilot and a custom MSDK app be connected at the same time? Do they expose any output?

### Takeaway
No. All evidence says only one app talks to the aircraft over the RC link at a time. DJI support states it explicitly for current platforms, and a developer report says the same for USB in general. GS Pro and DJI Pilot expose no plugin or telemetry stream that I could find. The custom app therefore has to either fly the mission itself, or take over the link after GS Pro starts the mission (Section 1 path).

### Cited Findings
- DJI SDK support: "MSDK App and DJI Fly cannot be used simultaneously because they will conflict with the drone access permissions. Therefore, only one app can access the drone data." (search-result quote from sdk-forum.dji.net; the page is login-gated and was not opened) — [DJI SDK forum: Can you run official app and MSDK app simultaneously on RC Pro or RC Plus](https://sdk-forum.dji.net/hc/en-us/articles/6029213604249-Can-you-run-official-app-and-MSDK-app-simultaneously-on-RC-Pro-or-RC-Plus)
- DJI support on Pilot 2 + MSDK (M300/M350): running both is "currently not possible", and they are "designed to have only one app communicating with the aircraft" (search-result summary, not opened). Applies to M300/M350, not M600. — [DJI SDK forum post 25832718888985](https://sdk-forum.dji.net/hc/en-us/community/posts/25832718888985-Can-MSDK-app-run-parallel-to-DJI-Pilot-2-for-M300-350-RTK-And-with-DJI-RC-Plus-Pro)
- A developer reported that only one application at a time can connect to the controller over USB and asked whether that comes from the Android Open Accessory protocol (search summary, unanswered). — [DJI SDK forum article 6079373639577](https://sdk-forum.dji.net/hc/en-us/articles/6079373639577)
- A DroneDeploy user: the mobile device can only be connected to one app through the USB cable at a time (user report). — [DroneDeploy forum](https://forum.dronedeploy.com/t/use-dji-googles-while-using-drone-deploy/10955)
- GS Pro is iOS/iPad only. 2.0.17 is the last update, and after that it "will enter maintenace phase, meaning no new updates or new device support." — [App Store: DJI GS Pro](https://apps.apple.com/us/app/dji-gs-pro/id1183717144)
- MSDK docs say only "An Android or iOS device can be connected to the remote controller through either USB or WiFi". M600 mobile connection is USB. No multi-app statement. — [MSDK Product Introduction](https://developer.dji.com/mobile-sdk/documentation/introduction/product_introduction.html)

### Inferences
- On iOS (required for GS Pro), the RC is an MFi External Accessory. Normally only one app holds the DJI protocol session at a time, and backgrounded apps lose it. I could not open Apple's EASession page to quote it, so this is unverified. Two iPads/phones cannot share one M600 RC USB port either. Some M600 setups use a second RC in master/slave mode with its own device: the slave RC has its own USB port, so a second device on the slave RC could run the custom app. This is an untested idea; whether the slave RC gets the SDK data link is unverified.
- The most robust design is for the custom MSDK app to plan, upload and fly the waypoint mission itself with `WaypointMissionOperator`. Then progress events are guaranteed, and the app can inject per-waypoint data to the payload. That replaces GS Pro or Pilot in the workflow.

### Gaps
- No GS Pro / DJI Pilot plugin, export or live-stream interface found. GS Pro only exports flight records after the fact (not researched in depth).
- Master/slave dual-RC behaviour for MSDK on M600 is unverified.

## 3. Passive mission indicators in FlightControllerState / keys

### Takeaway
`FlightControllerState.getFlightMode()` returns `GPS_WAYPOINT` ("WP/F-WP") while a waypoint mission flies, so any connected MSDK app can tell "a mission is running" with no mission ownership. No FlightControllerState field gives the current waypoint index. That comes only from the mission operator.

### Cited Findings
- `getFlightMode()` returns `FlightMode`. `GPS_WAYPOINT`: "GPS waypoint mode. Shown as WP/F-WP in DJI app." No mission or waypoint index exists in the state. The closest is `getFlightLogIndex()`, which is the flight-log index. `isFlying()` and `getAircraftLocation()` are also available. — [FlightControllerState (Android)](https://developer.dji.com/api-reference/android-api/Components/FlightController/DJIFlightController_DJIFlightControllerCurrentState.html)

### Inferences
- A passive app can combine `GPS_WAYPOINT` with position, and compare position to a known waypoint list, to work out progress itself. The ESP32 already gets position from the OSDK broadcast, so it could do the same matching onboard without a phone. The phone adds nothing here except `targetWaypointIndex`.
- The OSDK Open Protocol lists push data CMD 0x02 0x03 "Ground Station State" and 0x02 0x04 "Waypoint Event" (6-byte packages) — [OSDK Open Protocol](https://developer.dji.com/onboard-sdk/documentation/protocol-doc/open-protocol.html). These are probably emitted only for OSDK-uploaded missions, but if they fire for any mission, the ESP32 could read them directly without a phone. This is an adjacent lead for the OSDK researcher. Unverified.

### Gaps
- MSDK `DJIKey` / `MissionKey` for the waypoint index was not confirmed. I found no key exposing the index outside the operator.

## 4. M600 support in MSDK versions

### Takeaway
M600 / M600 Pro and A3 / A3 Pro are MSDK 4.x products, with v1 waypoint missions supported. MSDK v5 does not list the M600. Use the final MSDK 4.x release (4.16.x per my knowledge, not verified this session).

### Cited Findings
- The MSDK 4 product table lists Matrice 600 (cameras X3, X5, X5R, XT, Z3, Z30) and "Lightbridge 2 with A3 / N3". The "Vision System and Missions" table marks Matrice 600 and Matrice 600 Professional as supporting Waypoint, Hotpoint, Follow Me. — [MSDK Product Introduction](https://developer.dji.com/mobile-sdk/documentation/introduction/product_introduction.html)
- The DJI forum "Mobile SDK for Android 4 supports the following drones" lists "Matrice 600 & Matrice 600 Pro, RTK" and A3/N3 with Lightbridge 2 (search summary; page login-gated). — [DJI SDK forum 25516653720601](https://sdk-forum.dji.net/hc/en-us/articles/25516653720601)
- MSDK v5 aircraft lists (v5.13 notes, repo summary) cover M30, M300, M350, M4 series, M400, Mavic 3E/3TA, Mini 3/4 Pro, and do not include the M600. — [MSDK v5 release notes](https://developer.dji.com/doc/mobile-sdk-tutorial/en); [MSDK Android V5 repo index](https://docsearch.algolia.com/mcp/docs/repo/dji-sdk/mobile-sdk-android-v5)
- WaypointV2 is aimed at M300 RTK. The v1 operator is the one for the M600. — [DJIWaypointMissionOperator (iOS)](https://developer.dji.com/api-reference/ios-api/Components/Missions/DJIWaypointMissionOperator.html)

### Inferences
- v1 waypoint limits (99 waypoints max, `MAX_WAYPOINT_COUNT`, from memory and not verified this session) apply. GS Pro splits larger missions into multiple uploads, so `targetWaypointIndex` is per uploaded segment, not per survey.

### Gaps
- Exact last 4.x version and the current state of MSDK 4 app-key registration for new apps in 2026 are unverified. DJI may restrict new v4 app keys.

## 5. Relay feasibility: phone to ESP32

### Takeaway
There are two routes.

- **(a) Phone joins the ESP32 "SmartLauncher" AP directly.** This is trivial to code, but the phone is on the ground with the pilot, so the ESP32 Wi-Fi only reaches a few hundred metres line-of-sight at best. It drops out over a typical mapping mission. (Estimate; no source opened.)
- **(b) MSDK `sendDataToOnboardSDKDevice`.** This goes through the Lightbridge link to the A3 API port, arriving as OSDK push CMD 0x02/0x02. It is officially supported on M600/A3, but limited to 100 bytes per call, about 1 KB/s uplink. The onboard side most likely has to be an *activated* OSDK device (activation is not the same as control authority). Whether a passive UART listener that never activates still receives 0x02/0x02 is unverified.

### Cited Findings
- `sendDataFromMobileToOnboard:withCompletion:` supports "Matrice 100, Matrice 200 Series, Matrice 600, Matrice 600 Pro, A3, A3 Pro, and N3". "The size of the data cannot be greater than 100 bytes, and will be sent in 40 byte increments every 14ms." `isActive`: "Whether an Onboard SDK device is currently active on the platform." The newer `pipelines` are "only supported by Matrice 300 RTK". — [DJIOnboardSDKDevice (iOS)](https://developer.dji.com/api-reference/ios-api/Components/OnboardSDKDevice/DJIOnboardSDKDevice.html)
- Mobile-side API names: iOS `sendDataToOnboardSDKDevice:withCompletion:` and `flightController:didReceiveDataFromOnboardSDKDevice:` on `DJIFlightController`. Android `FlightController.sendDataToOnboardSDKDevice` and `setOnboardSDKDeviceDataCallback`. — [Mobile and Onboard SDK APIs](https://developer.dji.com/onboard-sdk/documentation/appendix/mobile-onboard-APIs.html)
- Bandwidth: "the current upstream (Mobile to onboard computer) bandwidth is around 1KB/s while the downstream (Onboard to Mobile Device) bandwidth is around 8KB/s". The onboard side receives "a CMD from Autopilot" with CMD Set 0x02, CMD ID 0x02, 1–100 bytes of user data. Onboard to mobile uses CMD Set 0x00, ID 0xFE. — [OSDK Data Transparent Transmission (M100 docs)](https://developer.dji.com/onboard-sdk/documentation/M100-Docs/application-development-guides/data-transparent-transmission.html)
- Open Protocol index: 0x02/0x02 "Data Transparent Transmission From Mobile SDK to Onboard SDK" sits in the push-data set, alongside broadcast. No activation or control requirement is stated for it. Obtain Control requires "enable API control" in Assistant. — [OSDK Open Protocol](https://developer.dji.com/onboard-sdk/documentation/protocol-doc/open-protocol.html)
- OSDK sample: sending OSDK to MSDK requires "the aircraft must be activated successfully firstly". Data travels "over the Lightbridge wireless communication link between the remote controller and aircraft". — [OSDK MSDK-comm sample doc](https://developer.dji.com/onboard-sdk/documentation/sample-doc/msdk-comm.html)
- A DJI forum post title says "Open Protocol and Data Transparent Transmission" were indicated as not available on M210 and M600. This conflicts with the iOS MSDK page listing M600/A3. The post is login-gated and its thread content and replies are unread. — [DJI SDK forum 360038614413](https://sdk-forum.dji.net/hc/en-us/community/posts/360038614413-Open-Protocol-and-Data-Transparant-Transmission-on-M210-and-M600)
- `MobileDevice::sendDataToMSDK` (newer OSDK 4) is listed for M210V2/M300. `MobileCommunication` is deprecated since OSDK 3.7. — [OSDK MobileDevice reference](https://developer.dji.com/onboard-api-reference/classDJI_1_1OSDK_1_1MobileDevice.html); [MobileCommunication reference](https://developer.dji.com/onboard-api-reference/classDJI_1_1OSDK_1_1MobileCommunication.html)

### Inferences
- The payload is tiny: index (1 B), reached flag, state, maybe lat/lon, about 10–20 B per event. 1 KB/s is ample.
- The receive path is push data (0x02 set) and needs no Obtain Control, so it fits "no control authority". The open question is activation. The ESP32 would have to send the activation command (CMD 0x00 0x01, with app ID/key) once. Activation is a session handshake, not a control takeover, so it may be acceptable within the project's constraint. If the ESP32 already receives broadcast without activating, test whether 0x02/0x02 also arrives without it (UNVERIFIED either way).
- Route (a), ESP32 Wi-Fi at 2.4 GHz from the air to a ground phone: line-of-sight range depends heavily on antenna and power, and is usually well under the M600 Lightbridge range of kilometres. Latency is low (ms) while in range. This is an estimate, no source opened. Route (b) latency is the Lightbridge datalink, likely tens of ms. Unverified.
- Simpler alternative worth weighing: skip the phone and let the ESP32 match broadcast position (plus flight mode/status if available in broadcast) against a preloaded waypoint list. Or check whether OSDK push 0x02 0x04 "Waypoint Event" fires for GS Pro missions.

### Gaps
- No primary statement on whether 0x02/0x02 transparent data is delivered to a non-activated device.
- The M600-specific forum contradiction (transparent transmission "not available") is unresolved.
- ESP32 air-to-ground range figures were not sourced.
