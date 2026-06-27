# Mubotics 7157 - FRC 2026 Robot Skeleton

A comprehensive FRC robot skeleton for Team 7157 (Mubotics), based on Team 254's 2025 codebase, updated for WPILib 2026.

## Features

- **Swerve Drive** with CTRE Phoenix 6 (v26)
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

## Dependencies (vendordeps)

- WPILib 2026.2.1
- Phoenix 6 (v26.1.0)
- AdvantageKit (v26.0.0)
- PathPlannerLib (2026.1.2)
- PhotonLib (v2026.1.1-rc-3)
- MapleSim

## Getting Started

1. Clone this repository
2. Open in VS Code with WPILib extension
3. Update team number in `.wpilib/wpilib_preferences.json`
4. Build with `./gradlew build`
5. Deploy with `./gradlew deploy`

## Simulation

Run simulation with:
```bash
./gradlew simulateJava
```

For replay mode:
```bash
./gradlew simulateJavaRelease -Preplay
```

## Credits

Based on [Team 254 (The Cheesy Poofs) 2025 Robot Code](https://github.com/Team254/FRC-2025-Public)
