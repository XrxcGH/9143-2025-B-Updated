# Team 9143 - 2025 Robot B Code (2026 Libraries)

This repository contains the code for Team 9143's 2025 second robot ("B" — KitBot roller + AlLow ground intake on the swerve chassis), updated to the 2026 WPILib/vendor libraries and brought in line with the [A robot](../9143-2025-A-main) codebase. The drivetrain runs Kraken X60s through CTRE Phoenix 6; the KitBot roller and AlLow mechanism run NEOs on Spark MAX controllers through REVLib.

---

## Robot Overview

- **Swerve Drivetrain**: CTRE Phoenix 6 swerve — mixed modules (front: SDS MK4i, back: SDS MK4n), all with L3+ drive gearing, all Kraken X60 drive/steer, CANcoders, and a Pigeon 2. The `Swerve` subsystem is identical to the A robot's; only the CANcoder offsets in `TunerConstants` are this robot's own.
- **KitBot Roller**: Single NEO/Spark MAX roller that ejects coral into the reef's L1 trough.
- **AlLow (Algae Low) Intake**: Pivoting ground intake (NEO pivot + NEO rollers) with closed-loop angle control that **holds position on the controller**.
- **Vision (dormant)**: The A robot's full Limelight stack (MegaTag2 pose fusion + AprilTag tracking) is wired in but inert — `VisionConstants.LIMELIGHT_NAMES` is empty until a camera is mounted.
- **Autonomous**: PathPlanner routines **and** Choreo trajectories, both selected from one dashboard chooser (default: *Center Drop*).

---

## Controls

### Driver (Xbox controller, port 0)
| Input | Action |
|---|---|
| Left stick | Field-centric translation (scaled to 75%) |
| Right stick X | Rotate |
| A (hold) | X-lock wheels (brake) |
| B (hold) | Point modules at left-stick direction |
| Y (press) | Toggle AprilTag vision tracking — *only bound once a Limelight is configured* |
| D-pad | Slow robot-centric nudges |
| Left bumper | Re-zero field-centric heading |
| Back/Start + X/Y | SysId characterization (testing only) |

### Operator (Xbox controller, port 1)
| Input | Action |
|---|---|
| Right stick Y | AlLow pivot manual control (**holds angle on release**) |
| D-pad down | AlLow deploy to intake angle (45°), rollers in |
| D-pad right | AlLow hold angle (15°), rollers stopped |
| D-pad up | AlLow stow (0°), rollers stopped |
| Left trigger (hold) | AlLow rollers intake |
| Right trigger (hold) | AlLow rollers eject |
| B | KitBot eject first piece (timed 0.5 s) |
| Y | KitBot eject stacked piece (timed 0.5 s) |
| X (hold) | KitBot re-align piece |
| A (hold) | KitBot jog piece |
| Start | Reset AlLow pivot encoder (works disabled) |

---

## Subsystems

### Swerve ([Swerve.java](src/main/java/frc/robot/subsystems/Swerve.java))
Identical to the A robot's Swerve: extends the Phoenix 6 `SwerveDrivetrain`, adds PathPlanner `AutoBuilder` configuration, vision pose fusion (with the required FPGA-to-Phoenix timestamp conversion), an AprilTag tracking command, SysId routines, and alliance-aware operator perspective. All driving (teleop and path following) uses **closed-loop velocity** so wheel speeds track the request regardless of battery sag. Top speed ~5.96 m/s at 12 V.

### KitBot ([KitBot.java](src/main/java/frc/robot/subsystems/KitBot.java))
One NEO/Spark MAX roller with voltage compensation and a stall current limit. All behavior is exposed as **command factories** (`ejectFirstPiece`, `ejectStackedPiece`, `realignPiece`, `jogPiece`) consumed by both the button bindings and PathPlanner `NamedCommands`, so the roller always stops when a command ends or is interrupted.

### AlLow ([AlLow.java](src/main/java/frc/robot/subsystems/AlLow.java))
Pivot arm (0° stowed → 60° max, encoder zeroed at the stow on boot) plus intake rollers. Angle moves run closed-loop **on the Spark MAX**, which latches the reference — so the arm keeps holding its angle no matter which command is scheduled, and roller-only commands never disturb the hold. A `sin(angle)` gravity feedforward (arb. voltage) is re-evaluated against the **measured** angle every loop. Manual stick control captures and holds the current angle the moment the stick is released. Soft limits on the controller bound travel in every control mode.

### Vision ([Vision.java](src/main/java/frc/robot/subsystems/Vision.java))
The A robot's vision stack with the goal resolver simplified for this robot (reef tags → bumpers flush and centered for the L1 trough; coral station tags → flush; barge/processor ignored). **Currently inert** — to enable: mount a Limelight (LLOS 2026.0+), add its short name and facing sign to `VisionConstants.LIMELIGHT_NAMES` / `LIMELIGHT_FACING_SIGNS`, and the driver Y-toggle, tracking command, and MegaTag2 pose fusion all activate.

### Telemetry ([Telemetry.java](src/main/java/frc/robot/Telemetry.java))
Publishes drivetrain state to NetworkTables (for AdvantageScope/Elastic) and CTRE SignalLogger (.hoot logs) at the odometry rate, plus Mechanism2d module visualizations.

### Dashboard ([Dashboard.java](src/main/java/frc/robot/Dashboard.java))
The only place that publishes dashboard data (plain NetworkTables under `/SmartDashboard` — no Shuffleboard API, which Elastic dropped). Field widget with live pose, match/battery/CAN vitals, AlLow and KitBot status, the AlLow arm `Mechanism2d`, the Elastic SwerveDrive widget, a *Zero AlLow Pivot* pit button, a low-resting-battery Alert, and the AdvantageKit structured outputs (pose, speeds, module states, 3D component poses).

---

## Dashboard (Elastic)

The tab layout (Setup / Autonomous / Teleop / Testing) lives in [elastic-layout.json](src/main/deploy/elastic-layout.json); the robot serves it over HTTP port 5800 and **switches Elastic to the right tab automatically** on mode changes (via [util/Elastic.java](src/main/java/frc/robot/util/Elastic.java)).

One-time setup per drive station laptop: install Elastic, connect to the robot, `File → Load Layout From Robot` (Ctrl+D). Edit the layout in Elastic, `File → Export Layout`, and commit it back to `src/main/deploy/elastic-layout.json`.

## Logging & analysis (AdvantageKit + AdvantageScope)

Logging runs through **AdvantageKit** (`Robot` extends `LoggedRobot`):

- **DriverStation data, joysticks, and console output** are captured automatically; `Dashboard.update()` records structured outputs every loop (robot `Pose2d`, `ChassisSpeeds`, module states/targets, AlLow angle/target, KitBot output).
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
- `./gradlew test` (also run by `build`): `RobotContainerTest` proves the whole robot wires up and the scheduler runs; `AlLowSimTest` proves the pivot's simulated closed loop reaches the intake angle **and holds it with the manual stick centered** — the regression net for the position-hold fix.
- `gradle.properties` pins Gradle to the **WPILib JDK**; the Android Studio/JetBrains runtime that `JAVA_HOME` points at ships an old `msvcp140.dll` that hard-crashes WPILib natives during tests. Remove/adjust on machines without `C:/Users/Public/wpilib/2026`.
- Tests fork one JVM per class (`forkEvery`) because simulated CAN devices reject duplicate IDs in one process.

## Other tools

- **Phoenix Tuner X**: CTRE device config/firmware, CANcoder offset calibration, hoot logs.
- **REV Hardware Client**: Spark MAX firmware and live AlLow gain tuning.
- **PathPlanner GUI**: edit paths/autos in `src/main/deploy/pathplanner`.
- **Choreo**: draw time-optimal trajectories into `src/main/deploy/choreo`; they appear in the auto chooser automatically (see [Autonomous](#autonomous-pathplanner--choreo)).
- Optional future addition: **URCL** vendordep to stream Spark MAX internals into AdvantageScope (`Logger.registerURCL(...)` is ready in AdvantageKit 26).

---

## Autonomous (PathPlanner + Choreo)

Two path-authoring tools feed the **same** auto chooser (`SmartDashboard/Auto Mode`), so the driver picks a PathPlanner auto **or** a Choreo trajectory from one dropdown.

### PathPlanner
- Autos live in `src/main/deploy/pathplanner/autos` (*Center Drop*, *Leave*, *Starting Left*, *Starting Right*), paths in `.../paths` (2025.X file format, which PathPlanner 2026 uses as well).
- The KitBot commands are registered as `NamedCommands` under their original names (`EjectFirstPieceCommand`, etc.) — the `.auto` files reference these strings.
- Path-following feedback gains live in `Constants.AutoConstants` (translation kP = 10, rotation kP = 7 — the gains this robot's autos were driven with).
- `FollowPathCommand.warmupCommand()` is scheduled at startup so the first path of auto starts without a stutter.
- `deleteOldFiles = true` in build.gradle removes stale paths from the roboRIO on deploy.
- Logging: AdvantageKit records everything to `.wpilog` files on the roboRIO (see *Logging & analysis* below), alongside CTRE's `.hoot` signal logs.

### Choreo
- Choreo trajectories live in `src/main/deploy/choreo` (`*.traj`). `RobotContainer.addChoreoAutos()` discovers every `.traj` at startup and adds it to the chooser as **"Choreo: &lt;name&gt;"** — exactly how PathPlanner autos are picked up. Draw a trajectory in the [Choreo](https://choreo.autos) app (create/open a project whose folder is `src/main/deploy/choreo`) and it appears automatically; a bad file is logged and skipped, never crashing robot code.
- **Following is done by PathPlanner**, not ChoreoLib: PathPlanner 2026 natively loads Choreo `.traj` files (`PathPlannerPath.fromChoreoTrajectory`), so Choreo autos run through the same `AutoBuilder` holonomic controller and `AutoConstants` gains (translation kP 10 / rotation kP 7) as the PathPlanner autos — one code path to tune. Each Choreo auto resets odometry to the trajectory's (alliance-flipped) start pose, then follows.
- **Why not the ChoreoLib vendordep?** ChoreoLib's latest release is 2025 (`frcYear: 2025`); GradleRIO 2026 rejects a mismatched-year vendordep outright (*"...will break at runtime"*), so importing the `.traj` through PathPlanner is the supported way to run Choreo on a 2026 project. Swap to ChoreoLib's own `AutoFactory` if/when a 2026 release ships.
- `Demo.traj` is a placeholder straight-line move (2 m → 3 m on X) so the feature is testable out of the box (`ChoreoTrajectoryTest` loads it through the real parser). Replace it with real trajectories.

---

## Pre-Competition Checklist

1. **Verify the CANcoder offsets in Tuner X** (wheels aligned straight forward).
2. **Verify the AlLow angle reading**: command 45° and check with a protractor. The conversion (13.334°/motor rotation, ~27:1 implied) is a carried-over placeholder — if it's wrong, every gain on top of it is meaningless.
3. **Verify the AlLow gravity model**: the feedforward assumes the arm is vertical at 0° (stowed) so gravity torque scales with sin(angle). If the stow orientation differs, adjust `gravityFeedforward()` in AlLow.java.
4. **Tune AlLow gains**: raise `ALLOW_PIVOT_kG` until the arm holds the intake angle with kP zeroed, then raise `ALLOW_PIVOT_kP` until tracking is crisp (`ALLOW_PIVOT_kI` stays 0 — kG replaces it).
5. **Test the autos** — path-following gains are unchanged from last season, but the library jump (PathPlanner 2025 → 2026) warrants a full re-run.
6. **Firmware**: 2026 firmware on all CTRE devices (TalonFX, CANcoder, Pigeon 2), current Spark MAX firmware via the REV Hardware Client, 2026 roboRIO image.

---

## Dependencies

- **WPILib 2026** (GradleRIO 2026.2.1)
- **Phoenix 6 (v26)**: CTRE swerve (Kraken X60/TalonFX, CANcoder, Pigeon 2). *(Phoenix 5 removed — nothing used it.)*
- **REVLib 2026**: Spark MAX controllers and NEO motors (KitBot, AlLow).
- **PathPlanner 2026**: Autonomous path generation and following — also loads **Choreo** `.traj` files, so Choreo is supported with no extra vendordep (ChoreoLib has no 2026 release yet).
- **AdvantageKit v26**: Logging framework — fully wired (`LoggedRobot`, `.wpilog` + RLOG live stream, logged auto chooser).
- **LimelightHelpers v1.14**: Limelight interface (requires LLOS 2026.0+ on the cameras, when mounted).

## Getting Started

1. **Clone this repository** to your local machine.
2. **Open the project** in WPILib VS Code 2026.
3. **Build and deploy** (`./gradlew deploy` or the WPILib "Deploy Robot Code" command).
