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
| Subsystem | Control logic, reads inputs via `Logger.processInputs()` | `Intake` |

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
