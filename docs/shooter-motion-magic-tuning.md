# Shooter Flywheel Motion Magic Velocity Tuning

The fixed, three-wheel shooter uses a servo hood. `ShooterAiming` points the chassis, and the hood
sets launch elevation. The PWM servos report commanded positions rather than measured angles.

Each TalonFX receives the same requested mechanism velocity through Phoenix 6
`MotionMagicVelocityTorqueCurrentFOC`. The TalonFX profiles velocity changes using its configured
acceleration, then applies the per-wheel slot 0 feedforward and PID gains with torque-current FOC.
The robot sends a zero acceleration override so the controller uses the configured Motion Magic
acceleration. Jerk is set to zero, giving a trapezoidal velocity profile. A zero velocity request,
stop, or disabled Driver Station stops all three motors. A disconnected motor signal also stops the
flywheels until all three signals are available again.

## Dashboard values

The source defaults and clamp ranges are:

| Dashboard key | Default | Range / unit | Purpose |
| --- | ---: | --- | --- |
| `Shooter/Flywheels/{Left,Middle,Right}/kS` | 9.0 | 0–12 A | Static torque-current feedforward. |
| `Shooter/Flywheels/{Left,Middle,Right}/kV` | 0.122 | 0–12 A/(rotation/s) | Velocity feedforward. |
| `Shooter/Flywheels/{Left,Middle,Right}/kA` | 0.5 | 0–12 A/(rotation/s²) | Profile acceleration feedforward. |
| `Shooter/Flywheels/{Left,Middle,Right}/kP` | 13 | 0–100 A/(rotation/s) | Corrects velocity error. |
| `Shooter/Flywheels/{Left,Middle,Right}/kI` | 0.0 | 0–100 A/((rotation/s)·s) | Corrects accumulated velocity error. |
| `Shooter/Flywheels/{Left,Middle,Right}/kD` | 0.0 | 0–100 A/(rotation/s²) | Corrects the rate of velocity error. |
| `Shooter/MotionMagic/AccelerationRotationsPerSecSq` | 100 | 1–1,000 rotations/s² | Limits profile acceleration and deceleration. |
| `Shooter/CurrentLimit/StatorAmps` | 120 | 1–120 A | Per-motor stator current limit. |
| `Shooter/CurrentLimit/TorqueAmps` | 120 | 0–120 A | Forward and reverse torque-current limit. |
| `Shooter/Tuning/MaxVelocityRPM` | 6,000 | 1–6,000 RPM | Software ceiling for velocity requests. |
| `Shooter/Tuning/VelocityToleranceRPM` | 500 | 1–500 RPM | Per-wheel shot-ready tolerance. |
| `Shooter/Tuning/TargetRPM` | 2,500 | 0–MaxVelocityRPM | Target for the dashboard run command. |
| `Shooter/Tuning/HoodAngleDeg` | 35 | 20–65 degrees | Hood angle used by dashboard tuning commands. |
| `Shooter/Characterization/RampVoltsPerSec` | 0.5 | 0.05–2.0 V/s | Open-loop characterization ramp rate. |
| `Shooter/Characterization/MaxVolts` | 6.0 | 0–6 V | Open-loop characterization ceiling. |

The existing per-wheel gain values are preserved. Phoenix applies these slot 0 gains to a
torque-current output, so their output units are now amps. The configured sensor-to-mechanism ratio
is 1:1, and all velocity requests and gains use mechanism rotations per second.

Changes to gains, current limits, and Motion Magic acceleration are sent to all three TalonFXs on
the next enabled subsystem loop. The maximum velocity setting clamps software requests. The hood
and flywheel readiness checks continue to gate normal feeding; flywheel recovery is handled by the
same profiled velocity loop throughout a shot.

## Tuning procedure

1. Secure the robot, remove game pieces, and keep people and tools clear of the flywheel path.
2. Confirm all three TalonFXs support the Pro-licensed torque-current FOC request. Check
   `Shooter/UnlicensedFeatureFault/Any` and the individual fault logs.
3. Start with a low target using `Shooter/Tuning/Run At Tunable Setpoint`. Verify each wheel's
   direction, velocity, error, voltage, and current before feeding a game piece.
4. Adjust Motion Magic acceleration to control how quickly the wheels ramp between targets. Leave
   jerk at zero unless a smooth S-curve is needed and the configured value is added to tuning.
5. Tune each wheel's `kS`, `kV`, and `kA` feedforward, then `kP`. Raise `kI` only to correct a
   persistent steady-state error; add `kD` only when measured response shows a need. Validate low,
   middle, and high targets after each change.
6. Run `Shooter/Tuning/Feedforward Characterization` to ramp the wheels at bounded voltage and
   record the per-wheel `kS`/`kV` fits. This characterization is open-loop and does not change the
   active velocity controller.
7. Tune the hood angle and shot setpoints separately, then validate feeding at controlled distances.

## Useful logs

Compare `Shooter/VelocitySetpointRPM` with `Shooter/LeftVelocityRPM`, `MiddleVelocityRPM`, and
`RightVelocityRPM`, plus each `*ErrorRPM` trace. `Shooter/MotionMagic/AccelerationRotationsPerSecSq`
shows the profile setting. `Shooter/FlywheelAtSetpoint` and `Shooter/AtSetpoint` show the flywheel
and combined hood readiness states. Voltage, current, connection, and
`Shooter/UnlicensedFeatureFault/*` logs help distinguish tuning problems from hardware faults.

## References

- [CTRE Phoenix 6 Motion Magic controls](https://pro.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/motion-magic.html)
  describes velocity profiling, acceleration configuration, and the zero-acceleration override.
- [CTRE manual tuning with TorqueCurrentFOC](https://pro.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/manual-pid-tuning.html)
  describes the output units and tuning process for current-output control.
- [CTRE Tuner X connection guide](https://pro.docs.ctr-electronics.com/en/stable/docs/tuner/connecting.html)
  documents device connection and preflight checks.
