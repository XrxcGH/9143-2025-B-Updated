# Team 9143 - 2025 Robot B Code (2026 Libraries)

This repository contains the code for Team 9143's second 2025 FRC (Reefscape) robot — "B", "Megalodon": a KitBot roller and the AlLow ground intake on the swerve chassis — running on the 2026 WPILib and vendor libraries. The drivetrain runs Kraken X60s through CTRE Phoenix 6; the KitBot roller and the AlLow mechanism run NEOs on Spark MAX controllers through REVLib.

The drivetrain, vision and alignment code is the [A robot's](https://github.com/XrxcGH/9143-2025-A-Updated) (same chassis), where it has been run on the real robot. **On this robot it has not**: there is no Limelight mounted, so the whole vision stack is inert, and everything that still needs the robot is collected in the [Pre-Competition Checklist](#pre-competition-checklist).

**Latest release:** [v1.0](https://github.com/XrxcGH/9143-2025-B-Updated/releases/tag/v1.0) — release notes and source archives; all releases are on the [Releases page](https://github.com/XrxcGH/9143-2025-B-Updated/releases).

---

## Robot Overview

- **Swerve Drivetrain**: CTRE Phoenix 6 swerve — mixed modules (front: SDS MK4i, back: SDS MK4n), all with L3+ drive gearing, all Kraken X60 drive/steer, CANcoders, and a Pigeon 2. The `Swerve` subsystem is the A robot's; the CANcoder offsets in `TunerConstants` are this robot's own.
- **KitBot Roller**: Single NEO/Spark MAX roller that ejects coral into the reef's L1 trough. It ejects out of the robot's **rear**; the coral is loaded from the coral station at the **front**.
- **AlLow (Algae Low) Intake**: Pivoting ground intake (NEO pivot + NEO rollers) with closed-loop angle control that **holds position on the controller**, and is held again every time the robot enables.
- **Vision (inert — no camera mounted)**: The A robot's Limelight stack (MegaTag pose fusion + robot-frame AprilTag alignment) with the goal resolver reduced to what this robot scores with. `VisionConstants.LIMELIGHT_NAMES` is empty, so nothing touches NetworkTables and the camera-only driver bindings do not exist. Enabling a camera is a **constants-only change** — see [Vision](#vision-visionjava).
- **Autonomous**: PathPlanner routines **and** Choreo trajectories, both selected from one dashboard chooser (default: *Center Drop*).

---

## Controls

Anything that drives the robot by itself is a *hold*, never a toggle; anything rare or dangerous is Test-mode only, or disabled-only behind a 1 s hold. [RobotContainer.java](src/main/java/frc/robot/RobotContainer.java) is the single place where every binding is made.

### Driver (Xbox controller, port 0)
| Input | Action |
|---|---|
| Left stick | Field-centric translation |
| Right stick X | Rotation |
| A (hold) | X-lock the wheels (brake) |
| Left bumper | **Driver heading zero**: the way the robot faces now becomes "forward" on the stick |
| D-pad | Slow robot-centric nudges (0.5 m/s), all 8 directions |
| B, Back/Start + X/Y | Point modules / SysId — **Test mode only** |
| **Left trigger (hold)** | **Align on the reef**: rear bumper flush and centered on the face in view, for the L1 eject. Release = sticks back instantly. *Bound only once a Limelight is configured* |
| **Right trigger (hold)** | **Align on the coral station**: front bumper flush, centered. *Bound only once a Limelight is configured* |
| Y | **Re-seed the pose heading** from the AprilTags in view (MegaTag1 fused for 2 s). The driver's own "forward" does not move. *Bound only once a Limelight is configured* |

- **Speed scaling.** Stick speed and rotation rate are scaled by the *Drive – Teleop Speed Scale* tunable (0.75 by default: 75 % of top speed, 0.75 rot/s).
- **What the align triggers aim at is the driver's choice**, not inferred: the KitBot has no piece sensor, so one trigger means "reef" and the other "coral station". Tags of the other class are ignored while a trigger is held, so a station tag in view cannot hijack a reef alignment.
- **Rumble**: the driver's controller buzzes steadily while an alignment is held **and** aligned — the cue to call for the eject.
- **Driver heading zero** moves only the driver's frame, never the pose estimator's heading, so it is safe at any time (point the robot away from you first). While disabled with no tag supplying a heading — always the case while no camera is mounted — it also seeds the pose heading to alliance-forward; **Back + left bumper** forces that seed at any time.
- **Heading re-seed (Y).** While enabled only MegaTag2 is fused, and MegaTag2 takes its heading *from* the pose, so it never corrects a pose heading that has drifted. Y fuses MegaTag1 — whose solve carries its own heading — for 2 s (`HEADING_RESEED_WINDOW_SECONDS`); `Vision/Reseeding Heading` shows the window. With no tag in view it does nothing. Inactive in Test mode, where Back / Start + Y are SysId bindings.
- **SysId** applies open-loop voltage steps to the drivetrain, so those bindings (and B, point modules) exist only when Test mode is selected on the Driver Station.

### Operator (Xbox controller, port 1)
| Input | Action |
|---|---|
| Right stick Y | AlLow pivot manual control (**holds on release**, where the moving arm can stop) |
| D-pad down | AlLow deploy to intake angle (45°), rollers in |
| D-pad right | AlLow hold angle (15°), rollers stopped |
| D-pad up | AlLow stow (0°), rollers stopped |
| Left trigger (hold) | AlLow rollers intake |
| Right trigger (hold) | AlLow rollers eject |
| B | KitBot eject first piece (timed 0.5 s) |
| Y | KitBot eject stacked piece (timed 0.5 s) |
| X (hold) | KitBot re-align piece |
| A (hold) | KitBot jog piece |
| Start, **held 1 s, disabled only** | Zero the AlLow pivot encoder (arm at its stow) |

---

## Subsystems

### Swerve ([Swerve.java](src/main/java/frc/robot/subsystems/Swerve.java))
The A robot's Swerve: extends the Phoenix 6 `SwerveDrivetrain`, adds PathPlanner `AutoBuilder` configuration (with the alliance flip set to the 2025 field size), vision pose fusion (with the required FPGA-to-Phoenix timestamp conversion and a spin-rate gate), the AprilTag alignment command, SysId routines, and the driver's heading frame. All driving (teleop, alignment and path following) uses **closed-loop velocity** so wheel speeds track the request regardless of battery sag. Top speed ~5.96 m/s at 12 V.

- **The driver's "forward" never moves because of vision.** Field-centric driving steers relative to the pose heading, which vision is allowed to correct. So the driver's forward is kept as a direction in the **raw gyro frame** (which only the gyro moves), and the operator perspective is recomputed every loop to cancel whatever vision or a pose reset did to the pose heading (`Swerve.periodic`). It changes only when the driver zeroes it (left bumper), when an auto resets the pose, when the alliance changes, or — until the driver has zeroed it — while disabled with a converged two-or-more-tag heading seed.
- **Autonomous start keeps a fresh vision heading**: `resetPoseForAuto` resets only the translation when a strong MegaTag1 seed is fresh and agrees with the path's nominal heading within 20°; otherwise (always, while there is no camera) it resets the full pose. Both PathPlanner and the Choreo autos go through it.

### KitBot ([KitBot.java](src/main/java/frc/robot/subsystems/KitBot.java))
One NEO/Spark MAX roller with voltage compensation and a stall current limit, run open-loop. All behavior is exposed as **command factories** (`ejectFirstPiece`, `ejectStackedPiece`, `realignPiece`, `jogPiece`) consumed by both the button bindings and PathPlanner `NamedCommands`, so the roller always stops when a command ends or is interrupted. There is no position or velocity loop, so there is nothing to hold on enable.

### AlLow ([AlLow.java](src/main/java/frc/robot/subsystems/AlLow.java))
Pivot arm (0° stowed → 60° max, encoder zeroed at the stow on boot) plus intake rollers.

- Angle moves run closed-loop **on the Spark MAX** (plain position control), which latches the reference — so the arm keeps holding its angle no matter which command is scheduled, and roller-only commands never disturb the hold.
- A `sin(angle)` gravity feedforward (arbitrary-feedforward volts) is re-evaluated against the **measured** angle every loop. Re-sending the same reference is harmless under plain position control. It would *not* be under REV MAXMotion, which restarts its profile from the measured state on every new setpoint — a note in the class says so for whoever changes the control mode. No kS term is configured (see the note beside `ALLOW_PIVOT_kG`).
- **Manual release holds where the arm can stop**: when the stick returns to center the target is the measured angle plus the stopping distance (velocity × `ALLOW_MANUAL_RELEASE_STOP_SECONDS` / 2), not the angle the moving arm is passing through, which would let it coast past and be dragged back.
- **Held on enable**: disabling cuts the output and drops the closed loop; `RobotContainer.enabledInit()` (from `Robot.disabledExit`) holds the arm at its resting angle, so a deployed arm does not hang on brake mode alone until the operator touches something.
- **Zeroing is disabled-only**: operator Start held 1 s, or the Setup tab's *Zero AlLow Pivot* button — both do nothing while enabled, because zeroing a deployed arm shifts the soft limits and every preset.
- Soft limits on the controller bound travel in every control mode.

### Vision ([Vision.java](src/main/java/frc/robot/subsystems/Vision.java))
**This robot has no Limelight**, so the three per-camera arrays in `VisionConstants` are empty and the subsystem is inert: `periodic()` returns at once, nothing is written to or read from NetworkTables, every getter reports "nothing seen", and the driver's align / re-seed bindings are not created. To enable a camera — a constants-only change:

1. Mount it (Limelight OS 2026.0+, AprilTag pipeline at index 0) and add its short name to `LIMELIGHT_NAMES`.
2. List the tag classes it may **align** on in `LIMELIGHT_TRACKING_CLASSES`: `REEF` on a **rear** camera, `CORAL_STATION` on a **front** camera (see *approach sides* below). A class listed on a camera facing the other way is ignored, with a warning at startup.
3. **Measure** the lens pose, enter it in `LIMELIGHT_POSES` and mark it `measured = true`. Until then the camera supplies no alignment target and no heading seed (it still feeds MegaTag2 translation from its web-UI pose).
4. Add Camera Stream / *Has Target* widgets for it in Elastic (`/CameraPublisher/limelight-<name>`, `Vision/<name> Has Target`).

The constructor refuses mismatched array lengths by name, so a half-filled-in camera is a clear startup error rather than an index exception in the robot loop.

**Pose fusion.** Sends the estimated heading to each Limelight every loop and fuses the returned **MegaTag2** poses with distance/tag-count-scaled confidence. While **disabled** (and during a driver re-seed window) it fuses **MegaTag1** instead, whose solve carries an absolute heading, so the pose heading is field-correct before the match starts.

- Each camera frame is fused **once** (the NT sample timestamp identifies a frame); the cameras' estimates are inserted oldest-first.
- Estimates are rejected when off-field, when a MegaTag2 solve averages more than 6 m to its tags, when a single-tag MegaTag1 solve is ambiguous (> 0.7) or far (> 3 m), and while the robot is spinning fast (> 2 rad/s, plus 0.2 s).
- Each camera's last fused pose is drawn on the Field widget.

**Alignment** works in the **robot frame**: each camera's primary tag is converted from Limelight camera space into "where is the tag relative to the robot center" using that camera's mounting pose (`Vision.tagPositionInRobotFrame`, pinned by `VisionGeometryTest`), so a rear camera, a yawed camera or an offset lens all drive the same loop with no per-camera mirroring. Empty or all-zero camera arrays, and a camera-space Z under 0.1 m, never become a "0 m away" target.

- Translation is **one P controller on the error vector**: speed from its length (with a 0.12 m/s floor), direction along it, clamped as one. Per-axis controllers snap the direction of travel each time an axis crosses its own deadband, which whips the swerve modules round.
- It stops when the forward and lateral errors (3 cm each — the L1 trough runs the width of a face) are inside their deadbands, and **stays stopped** until one passes 1.6× its deadband. Heading is a second P loop (1° deadband, same hysteresis) to the heading **square to the tag's face**.
- Everything is **slew-limited** (3 m/s², 6 rad/s², including the ramp-down when the tag is lost).
- "Told to move, not moving, nearly there" for 0.3 s counts as **arrived by contact** rather than stalling against the reef, with its own exit threshold.
- `Swerve.isAligned()` gates the driver's rumble and is available for gating a score.

**The servo never drives on a raw camera solve.** When a trigger is pressed the closest tag of the chosen class is **latched**. Each *new* camera frame is turned into a **field position** for it — using the pose the robot had **when the image was captured** (`samplePoseAt` at the frame timestamp minus pipeline + capture latency), not now — low-pass filtered (α = 0.3) behind an outlier gate (0.25 m, 3 frames in a row re-seed it), and every loop the tracker gets that point seen from the **current odometry pose**: smooth between frames and free of camera latency. While the latched tag is out of view (it always is in the last stretch) its position is **carried on odometry** for up to 1.5 s (`Vision/Target From Memory`). The latch is dropped at once if the alignment class changes, and nothing new is latched until the driver lets go and presses again.

**The square heading does not need a field-true gyro.** The tag's square heading comes from the 2025 AprilTag layout, but the pose heading is in whatever frame the gyro was last zeroed. So each frame's MegaTag1 solve is compared with the pose heading at capture, the difference is filtered (`Vision/Heading Offset`), and the square heading is converted into the pose estimator's own frame before the servo sees it.

**Tracking goals and approach sides** (where the tag should sit relative to the robot center; 2025 Reefscape tags):

| Tags | Goal |
|---|---|
| Reef (6–11, 17–22) | Tag `REEF_FLUSH_DISTANCE` **behind** the robot center (rear bumper flush with the reef base), **centered on the face**, square — the KitBot ejects into the L1 trough from there; the trough has no left / right branch |
| Coral stations (1, 2, 12, 13) | Tag `STATION_FLUSH_DISTANCE` **ahead** of the robot center (front bumper flush with the wall), centered, square |
| Barge (4, 5, 14, 15), processor (3, 16) | No goal — nothing on this robot scores there. They still feed pose estimation and the *Best Tag* readouts |

- Which end is driven up to each element (`REEF_APPROACH_REAR = true`, `STATION_APPROACH_REAR = false`) is this robot's **mechanism layout**, taken from the authored autos: every reef path ends with the robot's heading equal to the way the reef tag faces (backed up to it) and every station path ends facing the wall. `VisionGeometryTest` pins the aligner's square headings against those path end rotations. **Verify on the robot** that stick-forward leads with the end the coral is loaded from.
- The flush distances are geometry — half the bumper-to-bumper length (0.464 m, the robot size in `pathplanner/settings.json`) plus a little standoff, 0.47 m by default, **not measured on this robot**. `Vision/Distance` (forward, negative behind) and `Vision/Lateral` (positive left) report the same robot-frame numbers, so tune by pushing the robot into position and copying the magnitude into the tunable.

**Camera mounting (MegaTag camera poses).** `LIMELIGHT_POSES` holds each lens position and orientation and is pushed to the camera at startup, but only for cameras marked *measured*. Limelight's robot-space convention: origin at the frame center on the floor, **X forward, Y toward the robot's right** (opposite of WPILib), Z up; pitch positive = lens tilted up; yaw = lens heading (180° = rear-facing). After deploying, open `http://limelight-<name>.local:5801` and confirm the 3D preview shows the camera where it is, pointing the way it points; if not, flip the pitch or yaw sign in the constant.

**Heat and fan noise.** The LEDs are never turned on, and processing is **throttled while the robot is disabled** (one frame per 100 skipped — still ~1 solve/s for the pre-match heading seed), with full rate restored the instant it enables. **Selecting Test mode on the Driver Station lifts the throttle without enabling**, so the vision readouts are live when checking a camera on the bench.

### Telemetry ([Telemetry.java](src/main/java/frc/robot/Telemetry.java))
Publishes drivetrain state to NetworkTables (for AdvantageScope/Elastic) and CTRE SignalLogger (.hoot logs) at the odometry rate, plus Mechanism2d module visualizations.

### Dashboard ([Dashboard.java](src/main/java/frc/robot/Dashboard.java))
The only place that publishes dashboard data (plain NetworkTables under `/SmartDashboard` — no Shuffleboard API, which Elastic dropped). Field widget with live pose (and each camera's last fused pose), match/battery/CAN vitals, AlLow and KitBot status, the AlLow arm `Mechanism2d`, the Elastic SwerveDrive widget, the vision and alignment readouts, the *Zero AlLow Pivot* and *Reset Tunables* buttons, a low-resting-battery Alert, and the AdvantageKit structured outputs (pose, speeds, module states, 3D component poses).

---

## Dashboard (Elastic)

The tab layout (Setup / Autonomous / Teleop / Testing) lives in [elastic-layout.json](src/main/deploy/elastic-layout.json); the robot serves it over HTTP port 5800 and **switches Elastic to the right tab automatically** on mode changes (via [util/Elastic.java](src/main/java/frc/robot/util/Elastic.java)). Every widget's topic is published by the code whether or not a camera is configured — with none, the vision widgets simply read "nothing seen".

One-time setup per drive station laptop: install Elastic, connect to the robot, `File → Load Layout From Robot` (Ctrl+D). **Re-download it after deploying this version** — the vision, alignment and Tunables widgets appear only once the new layout is loaded. Edit the layout in Elastic, `File → Export Layout`, and commit it back to `src/main/deploy/elastic-layout.json`.

### Vision readouts
Two separate questions are answered on the dashboard, and they deliberately use different data.

**"What do the cameras see?"** — unfiltered, so it always agrees with the camera streams:

| Topic (under `/SmartDashboard`) | Meaning |
|---|---|
| `Vision/Best Tag` | ID of the **closest tag any camera reports**, −1 when none. No tag-class, camera-role or mounting-pose filter is applied |
| `Vision/Best Tag Camera` | Name of the camera that sees it ("" when none) |
| `Vision/Visible Tags` | Every tag ID per camera, e.g. `front: 12 \| rear: 18 19` |
| `Vision/<camera> Has Target` | Per-camera "sees a tag" light (one topic per configured camera; none today) |

**"What can the aligner use?"** — the closest tag a camera may **align** on (the tag is in that camera's class, the camera faces that class's approach side and its lens pose is measured), ignoring the alignment class and the latch so the readouts work with the robot pushed into position while disabled:

| Topic | Meaning |
|---|---|
| `Vision/Alignment Tag` | ID of that tag, −1 when none |
| `Vision/Distance`, `Vision/Lateral` | Its robot-frame position: forward (negative = behind) and left, meters |
| `Vision/TX`, `Vision/Square Heading` | Its horizontal angle in the camera, and the field heading square to its face |

`Vision/Alignment Tag` = −1 while `Vision/Best Tag` shows an ID means the tag is seen but is not one that camera aligns on.

**Alignment state** (while a trigger is held): `Swerve/Vision Tracking`, `Vision/Alignment Class` (REEF / CORAL_STATION / NONE), `Vision/Latched Tag`, `Vision/Target Visible`, `Vision/Target From Memory`, `Vision/Aligned`, `Vision/Forward Error`, `Vision/Lateral Error`, `Vision/Heading Error`, `Vision/Heading Offset`. **Heading seed**: `Vision/Heading Seed Fresh`, `Vision/Reseeding Heading`, `Vision/Auto Kept Heading`.

- A camera whose tag list has not changed for 10 s is ignored by the seen-tag readouts, because NetworkTables keeps the last value of a camera that lost power or its link.
- While disabled the cameras are throttled to about one solve per second, so the readouts lag the streams. **Select Test mode on the Driver Station** (no need to enable) to lift the throttle for bench checks.
- Where they are: *Best Tag* / *Align Tag* / *Visible Tags* / heading on **Setup**; *Vision Tracking* / *Aligned* / *Aligning On* / *Latched Tag* on **Teleop**; the alignment tag's TX / distance / lateral, the three servo errors and the Tunables on **Testing**.

### Live tuning without redeploying (Tunables)
The numbers that get dialed in with the robot in front of you live in [util/Tunables.java](src/main/java/frc/robot/util/Tunables.java), backed by **WPILib Preferences**: they appear in the Testing tab's *Tunables* widget, and the roboRIO **persists them to disk** — they survive reboots, power cycles, *and* code deploys. Every getter clamps its value to a sane range, so a typo on the dashboard cannot command something dangerous. Edits take effect on the next loop.

**Defaults vs. stored values.** The values in `Constants.java` are only the factory defaults; *Reset Tunables to Defaults* restores them. Because stored values survive a deploy, **changing a default in `Constants.java` does nothing on a robot that already has the key stored**. Bump `DEFAULTS_VERSION` in `Tunables.java` (which overwrites every tunable once at the next boot) or press *Reset Tunables*. When only a few defaults move, add a targeted migration block in `Tunables.init()` instead (the shape is shown there), so everything else tuned on the dashboard survives.

| Tunable | Default | What it sets |
|---|---|---|
| Drive – Teleop Speed Scale (0–1) | 0.75 | Fraction of top speed *and* of the 1 rot/s rotation rate at full stick; clamped to 0.05–1.0. Lower it (0.25) for indoor testing |
| Vision – Reef Flush Distance (m) | 0.47 | Robot center to the reef tag with the **rear** bumper flush. Taken by magnitude; clamped to 0.3–2.0 |
| Vision – Station Flush Distance (m) | 0.47 | Robot center to the station tag with the **front** bumper flush. Same clamp |
| Vision – Tracking Distance kP / Rotation kP | 1.5 / 0.06 | Alignment servo gains (m/s per m, rad/s per degree), clamped to 0 – 3× the default |

Deliberately **not** tunables: the AlLow pivot's Spark MAX gains and preset angles (written to the controller once at boot; try them live in the REV Hardware Client, then copy into `Constants`), and the AlLow conversion factor (a physical fact to verify, not a knob).

## Logging & analysis (AdvantageKit + AdvantageScope)

Logging runs through **AdvantageKit** (`Robot` extends `LoggedRobot`):

- **DriverStation data, joysticks, and console output** are captured automatically; `Dashboard.update()` records structured outputs every loop (robot `Pose2d`, `ChassisSpeeds`, module states/targets, AlLow angle/target, KitBot output, `Vision/BestTag`, `Vision/AlignmentTag`).
- `.wpilog` files land on a USB stick (`/U/logs`) if present, else `/home/lvuser/logs` (sim: `./logs`). Open in **AdvantageScope**.
- **Live streaming**: AdvantageScope → *Connect to Robot* with the **RLOG** source on **port 5810** (5800 is taken by the Elastic layout server). NT live viewing works too.
- The auto chooser is a `LoggedDashboardChooser`, so every log records which auto ran.
- CTRE's **SignalLogger** (`.hoot`) runs alongside for Phoenix signals and SysId.
- Note: full AdvantageKit **log replay** would additionally require IO-layer abstraction in every subsystem — this integration provides comprehensive logging, not deterministic replay.

**3D CAD animation**: `Dashboard` publishes `RobotState/ComponentPoses` (`Pose3d[]` — the AlLow arm) in the robot frame (X forward, Y left, Z up) every loop, so the arm animates from logs and in simulation. To attach the CAD in AdvantageScope's **3D Field** tab (one-time, per laptop):

1. Export the robot CAD to glTF with one file per moving group — `model.glb` (static chassis) and `model_0.glb` (AlLow arm).
2. Put them in a `Robot_9143B/` folder next to a `config.json` in AdvantageScope's user-assets directory (*Help → Show Assets Folder*):
   ```json
   {
     "name": "9143B",
     "rotations": [{ "axis": "x", "degrees": 90 }],
     "position": [0, 0, 0],
     "cameras": [],
     "components": [
       { "zeroedRotations": [{ "axis": "x", "degrees": 90 }], "zeroedPosition": [0, 0, 0] }
     ]
   }
   ```
   The single `components` entry maps to `Pose3d[]` index 0 (the AlLow arm). `zeroedPosition`/`zeroedRotations` describe the arm's CAD origin at rest — start at zero and adjust until it lines up (the `x: 90` rotation is the usual glTF Y-up → field Z-up fix).
3. Select the **9143B** model on the 3D robot, then bind its arm component to the `AdvantageKit/RealOutputs/RobotState/ComponentPoses` field (drag it onto the robot and choose the component/3D mapping). Live-over-NT and log-replay both use the same field.

The pivot offsets in `Dashboard.java` (`PIVOT_X_OFFSET`, `PIVOT_HEIGHT`) are placeholders — VERIFY against the CAD.

**Glass** works out of the box: `SmartDashboard/Field`, the `AlLow Mechanism` Mechanism2d, the command scheduler, all numeric topics for plotting, and `Telemetry`'s `DriveState` struct topics.

## Simulation & testing

- `./gradlew simulateJava` starts the robot in the WPILib **Sim GUI**: CTRE's swerve simulation drives like the real robot (keyboard mapping in `simgui-ds.json`), and the AlLow pivot has `SingleJointedArmSim` physics wired to the Spark MAX sim, so the preset buttons genuinely move the arm — watch it in the Mechanism2d or AdvantageScope.
- `./gradlew test` (also run by `build`) runs six test classes (24 tests):
  - `RobotContainerTest` — the whole robot wires up and the scheduler runs.
  - `AlLowSimTest` — against the simulated arm with a stepped clock: the closed loop reaches the intake angle **and holds it with the manual stick centered**.
  - `AlLowManualReleaseSimTest` — same simulated arm: a manual release mid-travel holds **where the arm stops** (ahead of the release angle), with the target set once.
  - `ControlsBindingTest` — presses simulated buttons on the real `RobotContainer`: the align triggers are holds and exist only with a camera configured, driver Y likewise, point-wheels / SysId only in Test mode, the diagonal D-pad nudges, the left bumper leaves the pose heading alone while enabled, every operator binding, the pivot zero needs *disabled* and a 1 s hold, and the arm is held on enable.
  - `VisionGeometryTest` — the camera-space → robot-frame conversion, the square headings against the authored paths' end rotations, the reef (rear) / station (front) goals, the tag classes, and the unfiltered *Best Tag* selection (including "no camera at all").
  - `ChoreoTrajectoryTest` — `Demo.traj` loads through the real parser.
- `gradle.properties` pins Gradle to the **WPILib JDK** (`C:/Users/Public/wpilib/2026/jdk`); another JDK on `JAVA_HOME` can hard-crash WPILib natives during tests. On macOS / Linux, or with WPILib installed elsewhere, override `org.gradle.java.home` in `~/.gradle/gradle.properties` (it takes precedence) or edit the line.
- Tests fork one JVM per class (`forkEvery`) because simulated CAN devices reject duplicate IDs in one process.

## Other tools

- **Phoenix Tuner X**: CTRE device config/firmware, CANcoder offset calibration, hoot logs. No Tuner X project file is kept in the repository — `generated/TunerConstants.java` is the source of truth (see the checklist).
- **REV Hardware Client**: Spark MAX firmware and live AlLow gain tuning.
- **PathPlanner GUI**: edit paths/autos in `src/main/deploy/pathplanner`.
- **Choreo**: draw time-optimal trajectories into `src/main/deploy/choreo`; they appear in the auto chooser automatically (see [Autonomous](#autonomous-pathplanner--choreo)).
- Optional future addition: **URCL** vendordep to stream Spark MAX internals into AdvantageScope (`Logger.registerURCL(...)` is ready in AdvantageKit 26).

---

## Autonomous (PathPlanner + Choreo)

Two path-authoring tools feed the **same** auto chooser (`SmartDashboard/Auto Mode`), so the driver picks a PathPlanner auto **or** a Choreo trajectory from one dropdown. If PathPlanner cannot be configured (`deploy/pathplanner/settings.json` missing or invalid) the chooser offers only "None", an error Alert is raised, and the rest of the robot still runs.

### PathPlanner
- Autos live in `src/main/deploy/pathplanner/autos` (*Center Drop*, *Leave*, *Starting Left*, *Starting Right*), paths in `.../paths` (2025.X file format, which PathPlanner 2026 uses as well). The reef paths end **backed up** to the reef (the KitBot ejects out of the rear); the station paths end facing the station.
- The alliance flip uses the **2025 field size** (17.548 × 8.052 m, set in `Swerve.configureAutoBuilder`); PathPlannerLib 2026 defaults to the 2026 field, which would mirror red-alliance paths about the wrong centerline.
- The KitBot commands are registered as `NamedCommands` under their original names (`EjectFirstPieceCommand`, etc.) — the `.auto` files reference these strings.
- Path-following feedback gains live in `Constants.AutoConstants` (translation kP = 10, rotation kP = 7 — the gains this robot's autos were driven with).
- `FollowPathCommand.warmupCommand()` is scheduled at startup so the first path of auto starts without a stutter.
- `deleteOldFiles = true` in build.gradle removes stale paths from the roboRIO on deploy.

### Choreo
- Choreo trajectories live in `src/main/deploy/choreo` (`*.traj`). `RobotContainer.addChoreoAutos()` discovers every `.traj` at startup and adds it to the chooser as **"Choreo: &lt;name&gt;"** — exactly how PathPlanner autos are picked up. Draw a trajectory in the [Choreo](https://choreo.autos) app (create/open a project whose folder is `src/main/deploy/choreo`) and it appears automatically; a bad file is logged and skipped, never crashing robot code.
- **Following is done by PathPlanner**, not ChoreoLib: PathPlanner 2026 natively loads Choreo `.traj` files (`PathPlannerPath.fromChoreoTrajectory`), so Choreo autos run through the same `AutoBuilder` holonomic controller and `AutoConstants` gains as the PathPlanner autos — one code path to tune. Each Choreo auto resets odometry to the trajectory's (alliance-flipped) start pose through `resetPoseForAuto`, then follows.
- **Why not the ChoreoLib vendordep?** ChoreoLib's latest release is 2025 (`frcYear: 2025`); GradleRIO 2026 rejects a mismatched-year vendordep outright, so importing the `.traj` through PathPlanner is the supported way to run Choreo on a 2026 project.
- `Demo.traj` is a placeholder straight-line move (2 m → 3 m on X) so the feature is testable out of the box. Replace it with real trajectories.

---

## Pre-Competition Checklist

None of the code in this version has been run on this robot. In order:

1. **Verify the CANcoder offsets in Tuner X** (wheels aligned straight forward) against `generated/TunerConstants.java`, which is the **source of truth**. The repository's old `tuner-project.json` was removed because it was stale: all four of its CANcoder offsets disagreed with `TunerConstants.java`, and it modelled every module with the MK4i steer ratio although the back modules are MK4n. Tuner X's generator assumes one module type, so do not regenerate the file — enter re-measured offsets by hand.
2. **Verify which end is "front"**: after a driver heading zero (left bumper), stick-forward must lead with the end the coral is loaded from, and the KitBot must eject out of the other end. `REEF_APPROACH_REAR` / `STATION_APPROACH_REAR` and every authored auto assume it.
3. **Drive-team controls changed**: left bumper is a *driver* heading zero (Back + LB also seeds the pose heading), B / SysId are Test-mode only, the D-pad nudges in 8 directions, and the stick speed comes from the *Teleop Speed Scale* tunable. Drive it once before a match.
4. **Verify the AlLow angle reading**: command 45° and check with a protractor. The conversion (13.334°/motor rotation, ~27:1 implied) is a carried-over placeholder — if it's wrong, every gain on top of it is meaningless.
5. **Verify the AlLow gravity model**: the feedforward assumes the arm is vertical at 0° (stowed) so gravity torque scales with sin(angle). If the stow orientation differs, adjust `gravityFeedforward()` in AlLow.java.
6. **Tune AlLow gains**: raise `ALLOW_PIVOT_kG` until the arm holds the intake angle with kP zeroed, then raise `ALLOW_PIVOT_kP` until tracking is crisp (`ALLOW_PIVOT_kI` stays 0 — kG replaces it).
7. **Check the AlLow hold behaviours**: enable with the arm deployed — it must stay put; release the manual stick mid-travel and watch `AlLow/Angle` against `AlLow/Target` — tune `ALLOW_MANUAL_RELEASE_STOP_SECONDS` (raise it if the arm comes back after a release, lower it if it creeps on). Confirm Start (held 1 s) zeroes the pivot only while disabled.
8. **Test the autos** — path-following gains are unchanged from last season, but the library jump (PathPlanner 2025 → 2026) and the 2025-field flip warrant a full re-run on both alliances.
9. **Firmware**: 2026 firmware on all CTRE devices (TalonFX, CANcoder, Pigeon 2), current Spark MAX firmware via the REV Hardware Client, 2026 roboRIO image.
10. **When a Limelight is mounted** (nothing below can be done before): fill in the three `VisionConstants` arrays and **measure the lens pose**; check the web-UI 3D preview; check *Best Tag* / *Visible Tags* against the stream in Test mode; push the robot flush against a reef face and a station wall and copy `|Vision/Distance|` into the two flush-distance tunables; then tune, on carpet, the servo values marked TUNE in `VisionConstants.TrackingGains` (the two kP tunables, the 0.12 m/s minimum speed, the deadbands, the contact thresholds) — the defaults are the A robot's, which shares the chassis but not the weight. Check the heading re-seed (Y) and that an auto started in view of two tags reports `Vision/Auto Kept Heading`.

---

## Dependencies

- **WPILib 2026** (GradleRIO 2026.2.1)
- **Phoenix 6 (v26)**: CTRE swerve (Kraken X60/TalonFX, CANcoder, Pigeon 2). *(Phoenix 5 removed — nothing used it.)*
- **REVLib 2026**: Spark MAX controllers and NEO motors (KitBot, AlLow).
- **PathPlanner 2026**: Autonomous path generation and following — also loads **Choreo** `.traj` files, so Choreo is supported with no extra vendordep (ChoreoLib has no 2026 release yet).
- **AdvantageKit v26**: Logging framework — fully wired (`LoggedRobot`, `.wpilog` + RLOG live stream, logged auto chooser).
- **LimelightHelpers v1.14** (source file in the project): Limelight interface (requires LLOS 2026.0+ on the cameras, when mounted).

## License

The team's code is released under the [MIT License](LICENSE). Files that come from elsewhere keep their own terms: the WPILib project template and build files ([WPILib-License.md](WPILib-License.md)), `LimelightHelpers.java` (Limelight), `util/Elastic.java` (the Elastic dashboard's license, linked in its header) and `generated/TunerConstants.java` (generated by CTRE Tuner X). The vendor libraries are downloaded by Gradle under their own licenses.

## Getting Started

1. Install the **WPILib 2026** tools, then **clone this repository**.
2. **Open the project** in WPILib VS Code 2026. On macOS / Linux, or with a non-default WPILib install, first point `org.gradle.java.home` at your WPILib JDK (see [Simulation & testing](#simulation--testing)).
3. **Build and test** with `./gradlew build`; try it without a robot with `./gradlew simulateJava`.
4. **Deploy** (`./gradlew deploy` or the WPILib "Deploy Robot Code" command), then load the dashboard layout: Elastic → `File → Load Layout From Robot`.
5. Before driving, zero the AlLow pivot with the arm at its stow (operator Start held 1 s while disabled, or the Setup tab button) and work through the [Pre-Competition Checklist](#pre-competition-checklist).
