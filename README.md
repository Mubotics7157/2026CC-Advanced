# Mubotics 7157 - FRC 2026 Competitve Concept

A comprehensive remake of the West Coast Products 2026 Competitive Concept code based on more effective and modern architecture and methods.

## Robot Overview
The 2026 FRC WCP Competitive Concept features a swerve drivetrain, drum shooting system, an articulating intake, and a conveyor indexing system for game piece feeding, and limelight based vision system combined with advanced odometry for localization.

## Architecture

### IO Abstraction Layer

Every subsystem follows a three-layer pattern enabling full simulation and log replay:

| Layer | Purpose | Example |
|---|---|---|
| `XxxIO` | Interface with `@AutoLog` inputs class | `IntakeIO` |
| `XxxIOReal` | Hardware implementation (Phoenix 6, CANcoders) | `IntakeIOReal` |
| `XxxIOSim` | Simulation implementation | `IntakeIOSim` |
| Subsystem | Control l# Mubotics 7157 — FRC 2026 "REBUILT" Robot

Competition robot software for **FIRST Robotics Competition 2026 — REBUILT**, built by **Team 7157, Mubotics**.

This is a ground-up rewrite of the West Coast Products (WCP) 2026 Competitive Concept. Rather than a straight port, it re-architects the concept around a modern, simulator-first, logging-driven design: an AdvantageKit IO abstraction layer on every subsystem, a central `RobotState`, a single coordinating `Superstructure` state machine, and an autonomous system that blends Choreo trajectories with a custom Local AD* pathfinder.

---

## Table of Contents

- [Robot Overview](#robot-overview)
- [The 2026 REBUILT Game](#the-2026-rebuilt-game)
- [Quick Start](#quick-start)
- [Architecture](#architecture)
  - [IO Abstraction Layer](#io-abstraction-layer)
  - [RobotState — Centralized State](#robotstate--centralized-state)
  - [Superstructure State Machine](#superstructure-state-machine)
- [Subsystems](#subsystems)
- [Shooter Aiming & Shoot-On-The-Move](#shooter-aiming--shoot-on-the-move)
- [Vision & Localization](#vision--localization)
- [Autonomous System](#autonomous-system)
- [Controls](#controls)
- [Simulation & Log Replay](#simulation--log-replay)
- [Bot Identity (Comp vs. Practice)](#bot-identity-comp-vs-practice)
- [Tuning Workflow](#tuning-workflow)
- [Package Structure](#package-structure)
- [Dependencies](#dependencies)
- [Build & Development](#build--development)
- [Contributing Conventions](#contributing-conventions)

---

## Robot Overview

The 2026 competition robot is a swerve-drive machine designed to intake, index, and launch the REBUILT game piece into the alliance Hub from a distance, with full shoot-on-the-move capability. Its major mechanisms:

- **Swerve Drivetrain** — Phoenix 6 (CTRE Gen 2) swerve with PathPlanner + Choreo trajectory following, vision-fused odometry, and heading-holding drive control.
- **Drum Shooter** — Three-wheel, fixed-angle (non-turret) shooter with a servo-actuated hood. Velocity and hood angle are driven by distance-based interpolation maps, and the chassis is auto-aimed at the goal while driving.
- **Articulating Intake** — A rotating "forebar" arm with rollers, PID/Motion-Magic position control, plus an agitation oscillation used mid-shot to feed pieces.
- **Conveyor Indexer** — A conveyor + feeder pair that moves pieces from the intake up into the shooter's feeder throat.
- **Vision** — A single Limelight running MegaTag 2 multi-tag pose estimation, fused into the drivetrain pose estimator. A PhotonVision-based simulator back-end mirrors the real camera in sim.
- **Status LEDs** — Operator feedback for robot/mechanism state.

---

## The 2026 REBUILT Game

The code targets the official 2026 **REBUILT** field (`AprilTagFields.k2026RebuiltWelded`) and game rules. Relevant game-specific logic lives in `frc.robot.lib.rebuilt`:

- **`RebuiltHubStatus`** — Computes whether the robot's own Hub is currently **active** (able to score) using the FMS game-specific message and the match clock. It models the autonomous window, the alternating teleop shifts, and the end-game window, falling back to "active" during the early teleop data-unavailable window per WPILib 2026 guidance. The `Superstructure` will only feed the shooter when `isOwnHubActive()` is true.
- **`FieldLocation`** — Alliance-aware, collision-safe navigation targets derived from the official tag layout: trenches, depot, and four **Hub staging poses** placed just outside the Hub perimeter so the robot can shoot without contacting it.

---

## Quick Start

```bash
git clone <repo-url>
cd <repo>
./gradlew build          # First build downloads WPILib + vendordeps
./gradlew simulateJava   # Launch desktop simulation
```

**Requirements:** JDK 17+ and network access on the first build (to fetch vendordeps). See [Build & Development](#build--development) for the full command reference.

---

## Architecture

### IO Abstraction Layer

Every subsystem follows a three-class IO pattern. Control logic never touches hardware directly — it reads an `@AutoLog`-generated inputs class that is populated by whichever IO implementation is injected at construction.

| Layer | Responsibility | Example |
|---|---|---|
| `XxxIO` | Interface + `@AutoLog` inputs struct (the data contract) | `IntakeIO` |
| `XxxIOReal` | Real hardware (TalonFX, CANcoders, servos, Limelight) | `IntakeIOReal` |
| `XxxIOSim` | Simulation (WPILib sim, MapleSim physics, PhotonVision) | `IntakeIOSim` |
| `Xxx` (subsystem) | Control logic; reads inputs via `Logger.processInputs()` | `Intake` |

`RobotContainer` selects Real vs. Sim implementations once, based on `RobotBase.isSimulation()`. This is what makes **full log replay** possible: replay feeds the exact same logged inputs back through the subsystem logic with no hardware present.

### RobotState — Centralized State

`RobotState` is a singleton that owns all shared, cross-subsystem robot state, so subsystems can be decoupled from each other:

- **Pose history** — a thread-safe, time-interpolatable buffer of field-to-robot poses with a 1.0 s lookback (`getFieldToRobot(timestamp)`), which is what lets vision timestamps line up with odometry history.
- **Velocity** — measured and desired chassis speeds, in both robot- and field-relative frames, plus yaw/pitch/roll rate and linear acceleration buffers for the same lookback window.
- **Pose prediction** — `getPredictedFieldToRobot(lookahead)` extrapolates the current pose forward using measured twist (used heavily by shoot-on-the-move).
- **Vision bookkeeping** — tracks the last accepted MegaTag timestamp to reject stale/duplicate observations, and supports an "exclusive tag" filter.
- **Alliance helpers** — `isRedAlliance()`, opponent-side checks, and the auto start time.
- Trajectory target/current poses for AdvantageScope overlays.

### Superstructure State Machine

`Superstructure` is the single coordinator for the intake, indexer, and shooter. Instead of scattering command compositions, drivers and autos simply request a `Goal`, and `Superstructure.periodic()` drives all three subsystems to satisfy it:

```
IDLE           intake stowed, shooter parked, indexer stopped
DEPLOYED_IDLE  intake deployed, everything else safe/stowed
INTAKING       arm deployed + rollers spinning, conveyor running
OUTTAKING      all rollers + conveyor + feeder reversed
SHOOTING       aim/velocity/hood from request, agitate intake, gated feeding
```

Key behaviors baked into `SHOOTING`:

- **Shot gating** — feeding only begins after the flywheels and hood are at setpoint continuously for a `SHOOTER_FEED_DELAY_SECONDS`, the aim solution is ready, **and** the alliance Hub is active. Once latched, feeding stays latched for the shot.
- **Agitation** — the intake forebar oscillates at a fixed period throughout a shot as a controlled agitator, returning to deployed when shooting ends.
- **Dumb mode** — a driver toggle that bypasses the setpoint/tolerance gate and feeds immediately on entry (useful for testing or when vision is unreliable). It is always reset to off at `teleopInit()`.
- **Tuning override** — a `ShooterTuningCommands` hook temporarily takes exclusive control of the shooter for characterization while keeping feeders stopped.

`atGoal()` reports whether the current goal is fully satisfied, and `waitUntilAtGoal()` turns that into a command — used by autos to chain "shoot then move."

---

## Subsystems

All subsystems live under `src/main/java/frc/robot/subsystems/`.

### Drive — `subsystems/drive/`
- `DriveSubsystem` wraps Phoenix 6 `CommandSwerveDrivetrain` with `DriveIO` Real/Sim variants. Real hardware reports odometry + module states; the sim variant uses `MapleSimSwerveDrivetrain` for physics-based sim.
- Tuner constants are **selected by bot identity** — `CompTunerConstants`, `PracTunerConstants`, and `SimTunerConstants` (see [Bot Identity](#bot-identity-comp-vs-practice)).
- Provides `sysIdQuasistatic` / `sysIdDynamic` routines and `addVisionMeasurement()` (with inverse-variance weighting) for vision fusion.
- `DriveViz` publishes 3D component poses for AdvantageScope.
- Swerve speeds/accel limits: max speed ~6.14 m/s, angular ~16.5 rad/s.

### Shooter — `subsystems/shooter/`
- `Shooter` runs a **three-wheel fixed drum** with a servo hood. The TalonFXs use `MotionMagicVelocityTorqueCurrentFOC` with per-wheel slot 0 feedforward and PID gains. One acceleration-limited velocity profile handles spool-up, speed changes, and recovery throughout a shot.
- `ShooterAiming` computes the shot: predicted shooter-exit position (accounting for robot velocity and rotation), a 3D distance to the target including height delta, and iterated **time-of-flight lookahead** to produce a heading, wheel velocity, and hood angle.
- `ShooterSetpoints` holds the distance→RPM, distance→hood-angle, and distance→time-of-flight interpolation maps (placeholder values — replace with characterized data), plus tunable RPM/hood offsets and the target geometry (official blue Hub center, target height, shooter exit offset).
- `ShooterIO` / `ShooterIOReal` / `ShooterIOSim` and `ShooterConstants` provide CAN IDs, gains, Motion Magic acceleration, and hood servo angle↔position conversion.

### Intake — `subsystems/intake/`
- Rotating **forebar arm** + rollers, position units in radians at the arm pivot.
- Arm uses Motion Magic / PID with gravity feedforward (`kG`); a deadzone tolerance prevents motor chatter at hold. Rollers run closed-loop velocity.
- Positions: **stowed**, **deployed**, and **agitate** (used by the superstructure during shots). All gains and motion constraints are `LoggedTunableNumber`s so they can be tuned live.

### Indexer — `subsystems/indexer/`
- Conveyor + feeder motors, each with independent velocity setpoints and PID.
- Exposes `isConveyorRunning()` / `isFeederRunning()` / `isRunning()` for superstructure goal checks.

### Vision — `subsystems/vision/`
See [Vision & Localization](#vision--localization).

### LED — `subsystems/led/`
- Status indicator LEDs.

---

## Shooter Aiming & Shoot-On-The-Move

The `ShootOnTheMoveCommand` is the primary shooting command and a good example of how the pieces fit together:

1. Sets `Superstructure` to `SHOOTING`.
2. Each loop, calls `ShooterAiming.calculate(robotState)` to produce a `ShotSolution` (target, compensated shooter position, robot heading, distance, time-of-flight, wheel velocity, hood angle, and readiness flags).
3. Feeds that solution into the superstructure via `setShootingRequest(...)`.
4. Drives with `FieldCentricFacingAngle` (a heading PID), letting the driver translate while the robot independently rotates to aim. Alliance is accounted for by flipping the drive frame.

Feeding only happens once the solution reports `readyToFeed()` (in range **and** heading aligned within tolerance at a low enough yaw rate) **and** the shooter is at setpoint **and** the alliance Hub is active.

---

## Vision & Localization

`VisionSubsystem` consumes a single Limelight (`VisionIOHardwareLimelight`) on the robot and a PhotonVision simulator (`VisionIOSimPhoton`) in sim, producing `VisionFieldPoseEstimate`s that are handed to `RobotState` and then fused into the drivetrain pose estimator.

The processing pipeline is deliberately defensive:

- **MegaTag 2 multi-tag** pose estimates are preferred. Estimates are **inverse-variance weighted** by a per-estimate quality factor, and a standard-deviation vector is built from the reported X/Y/Yaw deviations.
- **Gyro fusion fallback** — when only a single tag is visible, a pose can be reconstructed by combining the tag's field position with the odometry-implied rotation, and is only used when yaw rate is low (to avoid unreliable time-sync while spinning).
- **Rejection filters** — stale timestamps, invalid/non-finite data, high single-tag ambiguity, too-small tag area, unrealistic field norm or camera Z, large yaw disagreement with the prior pose, and inconsistent "exclusive tag" filters.
- All accepted/rejected decisions and estimates are logged under `Vision/` for offline debugging.

Drive-side, `DriveConstants` exposes disabled-vs-enabled process noise (std dev) used for the pose estimator so vision dominates while disabled and odometry dominates while driving.

---

## Autonomous System

Located in `frc.robot.auto`. The system was designed so **no file parsing or path construction happens at the start of autonomous** — everything is built and cached during `disabledPeriodic` when the dashboard selection changes.

- **`AutoModeSelector`** — LoggedDashboardChoosers for `DesiredMode`, plus choosers for a PathPlanner file, a Choreo trajectory, and target `FieldLocation`s. Modes:
  - `DO_NOTHING`
  - `SYSID_QUASISTATIC_FORWARD / _REVERSE`, `SYSID_DYNAMIC_FORWARD / _REVERSE`
  - `PATHPLANNER_AUTO` — load a named PathPlanner auto
  - `CHOREO_PATH` — follow a Choreo `.traj` file (with starting-pose support)
  - `PATHFIND_TO_LOCATION` — Local AD* pathfind to a selected `FieldLocation`
  - `ADAPTIVE_ROUTE` — a builder-composed 3-target route mixing intaking and shooting (see below)
- **`AdaptiveAutoBuilder`** — Builder-pattern autonomous routines targeting up to three `FieldLocation`s, with hooks (`afterFirstTarget`, etc.) that automatically inject an **intake dwell** at collection locations and a **stationary Hub shot** at Hub-staging locations. Chosen per-target from the dashboard.
- **`AutoUtil`** — Choreo trajectory loading/caching, alliance flipping, and PID-based trajectory following.
- **`AutoFactory`** — Shared factories for pathfinding to a location and to the Hub, used by both autos and teleop buttons.
- **Custom pathfinder** — the vendored `com.team254.lib.pathplanner` **Local AD\*** implementation supports different obstacle models for **teleop** vs. **auto** (`Pathfinding.setTeleopObstacles()` / `setAutoObstacles()`), and obstacle/cache management is handled in `Robot` disabled/auto/teleop init.
- **Choreo & PathPlanner assets** live in `src/main/deploy/pathplanner/` (including navgrids for pathfinding).

`Robot.disabledPeriodic` regenerates the autonomous command only when a dashboard selection changes, logs the starting pose, and `autonomousInit` resets odometry to that pose before scheduling.

---

## Controls

Driver input is abstracted behind two interfaces, so bindings are decoupled from physical controllers and easy to swap:

- **`IDriveControlBoard`** — continuous axes (throttle, strafe, rotation) and triggers: intake, outtake, shoot, toggle dumb mode, pathfind to Hub, stow intake, reset gyro.
- **`IButtonControlBoard`** — X-wheels, auto-align.

`GamepadDriveControlBoard` / `GamepadButtonControlBoard` implement these on Xbox controllers (with sim controller support via `CommandSimXboxController`). `ControlBoard` is a singleton that composes both.

**Default Xbox bindings** (from `GamepadDriveControlBoard` / `GamepadButtonControlBoard` / `RobotContainer`):

| Input | Action |
|---|---|
| Left stick | Translate (throttle / strafe, cubic response curve) |
| Right stick X | Rotate (squared response curve) |
| Left bumper (hold) | Intake |
| Right bumper (hold) | Outtake |
| Right trigger (hold) | Shoot on the move |
| D-pad Left | Pathfind to Hub |
| D-pad Right | Toggle "dumb mode" |
| D-pad Down | Manual stow intake |
| Start (hold) | X-wheels (with Back not pressed) |
| Back (hold) | Reset gyro / heading |

---

## Simulation & Log Replay

The robot is designed to be **fully simulatable** and **fully replayable**:

- **MapleSim** is used for physics-based swerve simulation (`useMapleSim = true`), with robot weight, bumper size, and wheel friction configured in `DriveConstants`.
- **PhotonVision** simulates the Limelight camera; `SimulatedRobotState` provides ground truth.
- WPILib Simulation GUI + Driver Station are launched automatically.

```bash
./gradlew simulateJava                    # Live simulation (also logs to .wpilog)
./gradlew simulateJavaRelease -Preplay    # Replay a recorded .wpilog (AdvantageKit)
./gradlew replayWatch                     # Watch for logs and auto-replay them
```

In `Robot.java`, AdvantageKit receivers are chosen by environment: real robot writes `.wpilog` (and NT4 when not on FMS); simulation writes both NT4 and `.wpilog`; replay sets a `WPILOGReader` and disables real-time timing. Results are viewable in **AdvantageScope** — the repo includes an `elastic-layout.json` and 3D component publishing for full-field and mechanism visualization.

---

## Bot Identity (Comp vs. Practice)

The robot auto-detects whether it is the **practice bot** by matching a hard-coded MAC address (`Constants.kPracticeBotMacAddress`, e.g. `00:80:2F:33:BF:BB`). This selects:

- The swerve **tuner constants** (`PracTunerConstants` vs. `CompTunerConstants`) in `DriveConstants`.
- Logged state (`SmartDashboard` "Is Practice Bot") so logs are self-identifying.

Simulation always uses `CompTunerConstants` (via `SimTunerConstants`). To adapt this for another robot, change the MAC address in `Constants.java`.

---

## Tuning Workflow

Most mechanism gains are `LoggedTunableNumber`s, meaning they can be edited live from AdvantageScope/NT during a session and are written to logs. Changing a gain at runtime triggers `hasAnyXxxChanged()` and a hardware config re-apply in `periodic()`.

- **Intake / Indexer** — PID, gravity, and Motion Magic constraints are all tunable (`IntakeConstants`, `IndexerConstants`).
- **Shooter** — distance→RPM, distance→hood, distance→time-of-flight maps in `ShooterSetpoints`, plus RPM and hood-angle offsets. `USE_TUNING_RPM` forces a manual test RPM.
- **Shooter characterization** — `ShooterTuningCommands.runAtTunableSetpoint()` and `feedforwardCharacterization()` are published to SmartDashboard and hold exclusive control of the shooter while running.
- **Drive** — translation, steering, and rotation SysId tests are selectable as auto modes; wheel
  radius / slip characterization is applied in the tuner constants. See the
  [drive SysId guide](docs/drive-sysid.md) for test order, logs, and setup.

---

Shooter Motion Magic velocity control, telemetry, and the recommended hooded-shooter tuning order are documented in [the shooter tuning guide](docs/shooter-motion-magic-tuning.md).

## Package Structure

```
src/main/java/
├── frc/robot/
│   ├── Robot.java              # LoggedRobot lifecycle; sim/replay wiring; auto/teleop/disabled
│   ├── RobotContainer.java     # Builds subsystems, picks Real vs Sim IO, binds controls
│   ├── RobotState.java         # Centralized pose/velocity/vision state + interpolation buffers
│   ├── Constants.java          # Robot dimensions, controller ports, bot identity
│   ├── auto/                   # AutoModeSelector, AdaptiveAutoBuilder, AutoUtil
│   ├── commands/               # ShootOnTheMove, DriveMaintainingHeading, pathfinding, tuning
│   ├── controlboard/           # IDriveControlBoard / IButtonControlBoard + gamepad impls
│   ├── factories/              # AutoFactory, IntakeFactory, ShooterFactory
│   ├── simulation/             # SimulatedRobotState, MapleSim integration
│   ├── subsystems/
│   │   ├── drive/              # Swerve drivetrain, IO, tuner constants, viz
│   │   ├── intake/             # Forebar arm + rollers
│   │   ├── indexer/            # Conveyor + feeder
│   │   ├── shooter/            # Drum shooter, aiming, setpoints
│   │   ├── superstructure/     # Goal state machine coordinating the above
│   │   └── vision/             # Limelight + PhotonVision IO, MegaTag fusion
│   ├── lib/
│   │   ├── drivers/            # CANDeviceId
│   │   ├── limelight/          # LimelightHelpers
│   │   ├── rebuilt/            # RebuiltHubStatus, FieldLocation (2026 game logic)
│   │   ├── subsystems/         # Reusable servo/motor IO wrappers, sim models
│   │   ├── time/               # RobotTime
│   │   └── util/               # Buffers, interpolating maps, math, tunables, etc.
│   └── util/                   # LoggedTunableNumber, simulation helpers
│
└── com/team254/lib/pathplanner/   # Vendored Local AD* pathfinder (254-style)
```

Deploy assets: `src/main/deploy/pathplanner/` (PathPlanner autos, navgrids) and `src/main/deploy/choreo/` (Choreo trajectories, if present).

---

## Dependencies

Vendored in `vendordeps/`:

| Library | Version | Role |
|---|---|---|
| WPILib | 2026 | Core framework (GradleRIO 2026.2.1) |
| Phoenix 6 | 26.3.0 | CTRE swerve, TalonFX, servos |
| AdvantageKit | 26.0.2 | Logging, replay, `@AutoLog` |
| PathPlannerLib | 2026.1.2 | Path following (vendored fork for Local AD*) |
| ChoreoLib | 2026.0.3 | Choreo trajectory loading |
| PhotonLib | v2026.3.4 | Vision simulation |
| MapleSim | 0.4.0-beta | Physics-based swerve sim |

Build plugins: `edu.wpi.first.GradleRIO` 2026.2.1, `com.peterabeles.gversion` (build metadata → `BuildConstants.java`), and `com.diffplug.spotless` (Google Java Format, AOSP style, enforced before compile).

---

## Build & Development

**Requires JDK 17+.** Code formatting runs automatically via Spotless before every compile, and the `BuildConstants` file (git SHA, build date) is generated as part of `compileJava`.

```bash
./gradlew build            # Compile + format + generate version file
./gradlew deploy           # Deploy to the RoboRIO
./gradlew deploy -PprofileMode   # Deploy with JMX profiling (connect via JVisualVM :1099)
./gradlew simulateJava     # Desktop simulation
./gradlew simulateJavaRelease -Preplay   # AdvantageKit log replay
./gradlew replayWatch      # Auto-replay logs as they appear
./gradlew test             # JUnit 5 tests
./gradlew spotlessApply    # Format code manually
./gradlew spotlessCheck    # Verify formatting
```

The RoboRIO JVM is configured for low, predictable GC pauses (fixed heap, SerialGC, `AlwaysPreTouch`); profiling mode is opt-in only because it slows the robot.

---

## Contributing Conventions

- **Never touch hardware directly** — add an `XxxIO` method and implement it in Real *and* Sim so replay/logging keep working.
- **Log generously** — use `Logger.recordOutput` / `@AutoLog` structs so every decision is debuggable offline.
- **Respect the IO pattern and `RobotState`** — read shared state from `RobotState`; don't couple subsystems to one another.
- **Prefer `Superstructure.Goal`** over ad-hoc command compositions for mechanism coordination.
- **Format before committing** — Spotless enforces Google Java Format (AOSP) and will reformat on build.
- **Alliance-aware everything** — use `FlippingUtil` / `FieldLocation.getPose(isRed)` so autos and aiming work on both sides.

---

*Team 7157 — Mubotics. Built for the 2026 REBUILT season.*ogic, reads inputs via `Logger.processInputs()` | `Intake` |

### Subsystems
- **Drive** (`subsystems/drive/`) - Phoenix 6 swerve drivetrain with PathPlanner + Choreo trajectory following. Tuner constants are selected by bot identity (comp vs practice bot, detected via MAC address).
- **Superstructure** (`subsystems/superstructure/`) - Top-level state machine coordinating intake, spindexer, and shooter using a `Goal` enum: `IDLE`, `DEPLOYED_IDLE`, `INTAKING`, `SHOOTING`, `CLIMBING`.
- **Shooter** (`subsystems/shooter/`) - Hooded shooter that uses positional data to automatically aim drivetrain in direction of goal as well as pistons that control the hood.
- **Intake** (`subsystems/intake/`) - Arm + rollers with PID position control.
- **Indexer** (`subsystems/indexer/`) - Game piece indexing with a set of conveyors responsible for indexing the gamepieces into the shooters feeder.
- **Vision** (`subsystems/vision/`) - AprilTag pose estimation via Limelight cameras (MegaTag 2 multi-tag fusion) with PhotonVision simulation support.
- **LED** (`subsystems/led/`) - Status indicator LEDs.

### RobotState (Centralized State)

`RobotState` is a singleton tracking all shared robot state:
- Robot pose with time-interpolatable buffer (1.0s lookback)
- Chassis speeds (measured and desired, robot and field relative)
- Mechanism positions (intake arm, turrets, hoods) for 3D AdvantageScope visualization
- Vision measurements with inverse-variance weighting
- Field zone tracking

### Autonomous System

Located in `frc.robot.auto`:

- **AutoModeSelector** - Dashboard choosers for mode, start side, intake locations, and shoot positions. Also includes SysId characterization modes.
- **ModularAutoBuilder** - Builder-pattern 2-cycle autonomous routines. Each cycle: intake at location -> drive to shoot position -> shoot. Supports Choreo paths (Center, Depot, Preload) and PathPlanner pathfinding (Outpost).
- **AutoUtil** - Choreo trajectory loading/caching, alliance flipping, PID-based trajectory following.
- Choreo `.traj` files live in `src/main/deploy/choreo/`.

### Control Board

Driver input is abstracted through interfaces:
- `IDriveControlBoard` - Throttle, strafe, rotation, plus triggers for intake/shoot/manual-stow/reset gyro/auto-align
- `IButtonControlBoard` - X-wheels, force stow intake, force lowest hood, flywheel/hood offset adjustments

Implementations (`GamepadDriveControlBoard`, `GamepadButtonControlBoard`) use Xbox controllers with sim controller support.

## Features

- **AdvantageKit** integration for logging and replay
- **PhotonVision** support for vision processing
- **PathPlanner** integration for autonomous path following
- **MapleSim** simulation support

## Package Structure

```
src/main/java/
├── frc/robot/                    # Main robot code
│   ├── subsystems/               # Robot subsystems
│   ├── commands/                 # Command classes
│   ├── controlboard/             # Driver controls
│   ├── auto/                     # Autonomous routines
│   └── lib/                      # Utility libraries
│       ├── drivers/
│       ├── util/
│       ├── subsystems/
│       └── ...
│
└── com/team254/lib/pathplanner/  # PathPlanner library (from 254)
```

## Dependencies

| Library | Version |
|---|---|
| WPILib | 2026 |
| Phoenix 6 | 26.1.1 |
| AdvantageKit | 2026 |
| PathPlannerLib | 2026 |
| ChoreoLib | 2026 |
| PhotonLib | 2026 |
| MapleSim | latest |

## Build & Development

**Requires Java 17+.** Code formatting (Google Java Format, AOSP style) runs automatically via Spotless before compilation.

```bash
./gradlew build                          # Compile and build
./gradlew deploy                         # Deploy to RoboRIO
./gradlew deploy -PprofileMode           # Deploy with JMX profiling
./gradlew simulateJava                   # Run simulation
./gradlew simulateJavaRelease -Preplay   # Replay mode (AdvantageKit log replay)
./gradlew test                           # Run tests (JUnit 5)
./gradlew spotlessApply                  # Apply code formatting manually
```

## Simulation

Run simulation with:
```bash
./gradlew simulateJava
```

For replay mode:
```bash
./gradlew simulateJavaRelease -Preplay
```
