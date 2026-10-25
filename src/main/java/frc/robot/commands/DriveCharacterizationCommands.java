package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Standard drivetrain characterization commands in the WPILib/Team 254 style.
 *
 * <p>Use {@link #wheelRadiusCharacterization} to measure the true wheel radius from the gyro (spin
 * the robot in place and compare how far the wheels rolled against the measured chassis rotation),
 * and {@link #feedforwardCharacterization} to fit the drive-axle kS/kV feedforward. Both print
 * their result to the console and record a summary to the log under {@code
 * Drive/Characterization/}.
 */
public final class DriveCharacterizationCommands {

    private static final double STEER_SETTLE_TIMEOUT_SECONDS = 3.0;
    private static final double STEER_STABLE_SECONDS = 0.25;
    private static final double STEER_VELOCITY_THRESHOLD_RAD_PER_SEC = 0.15;

    private static final DecimalFormat FORMAT = new DecimalFormat("0.00000");

    private DriveCharacterizationCommands() {}

    /**
     * Spins the robot in place and reports the effective wheel radius.
     *
     * <p>During a pure in-place spin the drivetrain commands each module tangentially at the speed
     * that matches its position, so the wheel point traces an arc of the module's drive-base
     * radius. The distance a wheel rolls equals {@code driveBaseRadius * chassisAngle}, and that
     * distance is also {@code wheelAngle * trueRadius}. Solving gives {@code trueRadius =
     * driveBaseRadius * chassisAngle / wheelAngle}.
     *
     * @param drive The drivetrain.
     * @param rotations Number of full robot rotations to spin (sign selects direction).
     * @param maxAngularVelocityRadPerSec Angular velocity used for the spin.
     */
    public static Command wheelRadiusCharacterization(
            DriveSubsystem drive, double rotations, double maxAngularVelocityRadPerSec) {
        if (!Double.isFinite(rotations) || rotations == 0.0) {
            throw new IllegalArgumentException("rotations must be finite and nonzero");
        }
        if (!Double.isFinite(maxAngularVelocityRadPerSec) || maxAngularVelocityRadPerSec <= 0.0) {
            throw new IllegalArgumentException(
                    "maxAngularVelocityRadPerSec must be finite and positive");
        }
        double targetAngleRadians = 2.0 * Math.PI * Math.abs(rotations);
        double direction = rotations < 0 ? -1.0 : 1.0;
        ChassisSpeeds spinSpeeds =
                new ChassisSpeeds(0.0, 0.0, direction * maxAngularVelocityRadPerSec);

        return new Command() {
            {
                addRequirements(drive);
            }

            private final Timer settleTimer = new Timer();
            private final Timer stableTimer = new Timer();
            private double lastGyroRadians;
            private double[] lastWheelAngle = new double[0];
            private double[] accumulatedWheelAngle = new double[0];
            private double accumulatedChassisAngle;
            private boolean baselineCaptured;
            private boolean stableTimerStarted;
            private boolean settleTimedOut;

            @Override
            public void initialize() {
                drive.runVelocity(new ChassisSpeeds());
                settleTimer.restart();
                stableTimer.stop();
                stableTimer.reset();
                stableTimerStarted = false;
                baselineCaptured = false;
                settleTimedOut = false;
                accumulatedChassisAngle = 0.0;
                lastWheelAngle = new double[0];
                accumulatedWheelAngle = new double[0];
            }

            @Override
            public void execute() {
                // Ignore initial module slewing so steering-coupling motion cannot bias the radius.
                drive.runVelocity(spinSpeeds);
                if (!baselineCaptured) {
                    if (areModulesSettled(drive.getModuleSteerVelocitiesRadPerSec())) {
                        if (!stableTimerStarted) {
                            stableTimer.restart();
                            stableTimerStarted = true;
                        } else if (stableTimer.hasElapsed(STEER_STABLE_SECONDS)) {
                            captureBaseline();
                            baselineCaptured = true;
                            stableTimer.stop();
                        }
                    } else {
                        stableTimer.stop();
                        stableTimer.reset();
                        stableTimerStarted = false;
                    }

                    if (!baselineCaptured && settleTimer.hasElapsed(STEER_SETTLE_TIMEOUT_SECONDS)) {
                        settleTimedOut = true;
                    }
                    return;
                }

                Rotation2d gyro = drive.getRotation();
                accumulatedChassisAngle +=
                        MathUtil.angleModulus(gyro.getRadians() - lastGyroRadians);
                lastGyroRadians = gyro.getRadians();

                double[] positions = drive.getWheelRadiusCharacterizationPositions();
                int count = Math.min(accumulatedWheelAngle.length, positions.length);
                for (int i = 0; i < count; i++) {
                    accumulatedWheelAngle[i] += positions[i] - lastWheelAngle[i];
                    lastWheelAngle[i] = positions[i];
                }
            }

            @Override
            public boolean isFinished() {
                return settleTimedOut
                        || (baselineCaptured
                                && Math.abs(accumulatedChassisAngle) >= targetAngleRadians);
            }

            @Override
            public void end(boolean interrupted) {
                drive.runVelocity(new ChassisSpeeds());
                settleTimer.stop();
                stableTimer.stop();
                if (interrupted) {
                    System.out.println("Wheel radius characterization interrupted.");
                } else if (settleTimedOut) {
                    System.out.println(
                            "Wheel radius characterization failed: modules did not settle.");
                } else if (baselineCaptured
                        && Math.abs(accumulatedChassisAngle) >= targetAngleRadians) {
                    finishWheelRadius(drive, accumulatedChassisAngle, accumulatedWheelAngle);
                }
            }

            private void captureBaseline() {
                lastGyroRadians = drive.getRotation().getRadians();
                double[] positions = drive.getWheelRadiusCharacterizationPositions();
                lastWheelAngle = positions.clone();
                accumulatedWheelAngle = new double[positions.length];
                accumulatedChassisAngle = 0.0;
            }
        }.withName("Drive Characterization: Wheel Radius");
    }

    private static boolean areModulesSettled(double[] steerVelocitiesRadPerSec) {
        if (steerVelocitiesRadPerSec == null || steerVelocitiesRadPerSec.length == 0) {
            return false;
        }
        for (double velocity : steerVelocitiesRadPerSec) {
            if (!Double.isFinite(velocity)
                    || Math.abs(velocity) > STEER_VELOCITY_THRESHOLD_RAD_PER_SEC) {
                return false;
            }
        }
        return true;
    }

    private static void finishWheelRadius(
            DriveSubsystem drive, double chassisAngle, double[] wheelAngles) {
        int valid = 0;
        double sumWheelAngle = 0.0;
        for (double angle : wheelAngles) {
            if (Math.abs(angle) > 1e-6) {
                sumWheelAngle += Math.abs(angle);
                valid++;
            }
        }
        if (valid == 0 || Math.abs(chassisAngle) < 1e-6) {
            System.out.println("Wheel radius characterization skipped: no motion recorded.");
            return;
        }

        double avgWheelAngle = sumWheelAngle / valid;
        double driveBaseRadius = drive.getDriveBaseRadiusMeters();
        // trueRadius = driveBaseRadius * chassisAngle / wheelAngle
        double measuredRadius = driveBaseRadius * Math.abs(chassisAngle) / avgWheelAngle;
        double nominalRadius = drive.getWheelRadiusMeters();

        Logger.recordOutput("Drive/Characterization/Wheel/AvgWheelAngleRad", avgWheelAngle);
        Logger.recordOutput("Drive/Characterization/Wheel/ChassisAngleRad", chassisAngle);
        Logger.recordOutput("Drive/Characterization/Wheel/MeasuredRadiusMeters", measuredRadius);
        Logger.recordOutput("Drive/Characterization/Wheel/NominalRadiusMeters", nominalRadius);
        Logger.recordOutput(
                "Drive/Characterization/Wheel/RadiusScaleFactor", measuredRadius / nominalRadius);
        System.out.println(
                "Wheel radius characterization: measuredRadius="
                        + FORMAT.format(measuredRadius)
                        + " m (nominal="
                        + FORMAT.format(nominalRadius)
                        + " m, scale="
                        + FORMAT.format(measuredRadius / nominalRadius)
                        + ")");
    }

    /**
     * Ramps the drive axle through a bounded voltage and fits the kS/kV feedforward from the logged
     * velocity samples.
     *
     * @param drive The drivetrain.
     * @param rampVoltsPerSec Voltage ramp rate.
     * @param maxVolts Maximum voltage to apply.
     */
    public static Command feedforwardCharacterization(
            DriveSubsystem drive, double rampVoltsPerSec, double maxVolts) {
        if (!Double.isFinite(rampVoltsPerSec) || rampVoltsPerSec <= 0.0) {
            throw new IllegalArgumentException("rampVoltsPerSec must be finite and positive");
        }
        if (!Double.isFinite(maxVolts) || maxVolts <= 0.0 || maxVolts > 12.0) {
            throw new IllegalArgumentException("maxVolts must be finite and in (0, 12]");
        }
        Timer timer = new Timer();
        List<Double> velocitySamples = new ArrayList<>();
        List<Double> voltageSamples = new ArrayList<>();

        Command rampAndSample =
                Commands.run(
                                () -> {
                                    double voltage =
                                            Math.min(
                                                    maxVolts,
                                                    Math.max(0.0, timer.get() * rampVoltsPerSec));
                                    drive.runCharacterization(voltage);
                                    double velocity = drive.getFFCharacterizationVelocity();
                                    double appliedVoltage = drive.getAverageDriveAppliedVoltage();
                                    velocitySamples.add(velocity);
                                    voltageSamples.add(appliedVoltage);
                                    Logger.recordOutput(
                                            "Drive/Characterization/FF/Voltage", appliedVoltage);
                                    Logger.recordOutput(
                                            "Drive/Characterization/FF/VelocityRadPerSec",
                                            velocity);
                                },
                                drive)
                        .until(() -> timer.get() >= maxVolts / Math.max(0.05, rampVoltsPerSec));

        return Commands.sequence(
                        Commands.runOnce(
                                () -> {
                                    velocitySamples.clear();
                                    voltageSamples.clear();
                                    drive.runCharacterization(0.0);
                                },
                                drive),
                        Commands.runOnce(timer::restart),
                        rampAndSample)
                .finallyDo(
                        interrupted -> {
                            drive.runCharacterization(0.0);
                            if (!interrupted) {
                                printFit(velocitySamples, voltageSamples);
                            } else {
                                System.out.println(
                                        "Drive kS/kV characterization interrupted; fit discarded.");
                            }
                        })
                .withName("Drive Characterization: Feedforward");
    }

    private static void printFit(List<Double> velocitySamples, List<Double> voltageSamples) {
        int count = Math.min(velocitySamples.size(), voltageSamples.size());
        if (count < 10) {
            System.out.println("Drive kS/kV characterization failed: only " + count + " samples");
            return;
        }
        double sumX = 0.0;
        double sumY = 0.0;
        double sumXY = 0.0;
        double sumX2 = 0.0;
        for (int i = 0; i < count; i++) {
            double x = velocitySamples.get(i);
            double y = voltageSamples.get(i);
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumX2 += x * x;
        }
        double denominator = count * sumX2 - sumX * sumX;
        if (Math.abs(denominator) < 1e-9) {
            System.out.println("Drive kS/kV characterization failed: no velocity range");
            return;
        }
        double kS = (sumY * sumX2 - sumX * sumXY) / denominator;
        double kV = (count * sumXY - sumX * sumY) / denominator;
        Logger.recordOutput("Drive/Characterization/FF/kS", kS);
        Logger.recordOutput("Drive/Characterization/FF/kV", kV);
        System.out.println(
                "Drive characterization: kS="
                        + FORMAT.format(kS)
                        + " V, kV="
                        + FORMAT.format(kV)
                        + " V/(rad/s), samples="
                        + count);
    }
}
