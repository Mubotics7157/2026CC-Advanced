package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Goal;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** Dashboard commands for flywheel setpoint tuning and feedforward characterization. */
public final class ShooterTuningCommands {
    private static final double HARD_MAX_CHARACTERIZATION_VOLTS = 6.0;

    private ShooterTuningCommands() {}

    /** Runs the tunable dashboard RPM and hood angle while preventing the indexer from feeding. */
    public static Command runAtTunableSetpoint(Shooter shooter, Superstructure superstructure) {
        return Commands.run(
                        () -> {
                            superstructure.setShooterTuningOverride(true);
                            superstructure.setGoal(Goal.IDLE);
                            shooter.setLaunchActive(false);
                            shooter.setVelocity(
                                    Units.rotationsPerMinuteToRadiansPerSecond(
                                            ShooterConstants.getTuningVelocityRPM()));
                            shooter.setHoodAngle(
                                    Units.degreesToRadians(
                                            ShooterConstants.getTuningHoodAngleDegrees()));
                        },
                        shooter,
                        superstructure)
                .finallyDo(
                        () -> {
                            shooter.stop();
                            superstructure.setShooterTuningOverride(false);
                            superstructure.setGoal(Goal.IDLE);
                        })
                .withName("Shooter Tuning: Run at Dashboard RPM");
    }

    /** Tracks the dashboard hood angle while preventing the superstructure from parking it. */
    public static Command moveHoodToTunableAngle(Shooter shooter, Superstructure superstructure) {
        return Commands.run(
                        () -> {
                            superstructure.setShooterTuningOverride(true);
                            superstructure.setGoal(Goal.IDLE);
                            shooter.setLaunchActive(false);
                            shooter.setHoodAngle(
                                    Units.degreesToRadians(
                                            ShooterConstants.getTuningHoodAngleDegrees()));
                        },
                        shooter,
                        superstructure)
                .finallyDo(
                        () -> {
                            shooter.stop();
                            superstructure.setShooterTuningOverride(false);
                            superstructure.setGoal(Goal.IDLE);
                        })
                .withName("Shooter Hood: Track Dashboard Angle");
    }

    /** Ramps all three wheels up to a bounded voltage and prints per-wheel kS/kV fits. */
    public static Command feedforwardCharacterization(
            Shooter shooter, Superstructure superstructure) {
        Timer timer = new Timer();
        List<Double> leftVelocity = new ArrayList<>();
        List<Double> middleVelocity = new ArrayList<>();
        List<Double> rightVelocity = new ArrayList<>();
        List<Double> leftAppliedVoltage = new ArrayList<>();
        List<Double> middleAppliedVoltage = new ArrayList<>();
        List<Double> rightAppliedVoltage = new ArrayList<>();

        Command rampAndSample =
                Commands.run(
                                () -> {
                                    double voltage =
                                            Math.min(
                                                    maxCharacterizationVolts(),
                                                    Math.max(
                                                            0.0,
                                                            timer.get()
                                                                    * characterizationRampRate()));
                                    shooter.runVoltage(voltage);
                                    leftVelocity.add(
                                            Math.abs(
                                                    Units.radiansToRotations(
                                                            shooter.getLeftVelocityRadPerSec())));
                                    middleVelocity.add(
                                            Math.abs(
                                                    Units.radiansToRotations(
                                                            shooter.getMiddleVelocityRadPerSec())));
                                    rightVelocity.add(
                                            Math.abs(
                                                    Units.radiansToRotations(
                                                            shooter.getRightVelocityRadPerSec())));
                                    leftAppliedVoltage.add(
                                            Math.abs(shooter.getLeftAppliedVoltage()));
                                    middleAppliedVoltage.add(
                                            Math.abs(shooter.getMiddleAppliedVoltage()));
                                    rightAppliedVoltage.add(
                                            Math.abs(shooter.getRightAppliedVoltage()));
                                },
                                shooter,
                                superstructure)
                        .until(
                                () ->
                                        timer.get()
                                                >= maxCharacterizationVolts()
                                                        / characterizationRampRate());

        return Commands.sequence(
                        Commands.runOnce(
                                () -> {
                                    leftVelocity.clear();
                                    middleVelocity.clear();
                                    rightVelocity.clear();
                                    leftAppliedVoltage.clear();
                                    middleAppliedVoltage.clear();
                                    rightAppliedVoltage.clear();
                                    superstructure.setShooterTuningOverride(true);
                                    superstructure.setGoal(Goal.IDLE);
                                    shooter.setLaunchActive(false);
                                    shooter.stop();
                                },
                                shooter,
                                superstructure),
                        Commands.runOnce(timer::restart),
                        rampAndSample)
                .finallyDo(
                        interrupted -> {
                            shooter.stop();
                            superstructure.setShooterTuningOverride(false);
                            superstructure.setGoal(Goal.IDLE);
                            if (!interrupted) {
                                printFit("Left", leftVelocity, leftAppliedVoltage);
                                printFit("Middle", middleVelocity, middleAppliedVoltage);
                                printFit("Right", rightVelocity, rightAppliedVoltage);
                            } else {
                                System.out.println(
                                        "Shooter feedforward characterization interrupted; fit discarded.");
                            }
                        })
                .withName("Shooter Tuning: Feedforward Characterization");
    }

    private static double maxCharacterizationVolts() {
        return Math.min(
                HARD_MAX_CHARACTERIZATION_VOLTS, ShooterConstants.getCharacterizationMaxVolts());
    }

    private static double characterizationRampRate() {
        return ShooterConstants.getCharacterizationRampRateVoltsPerSec();
    }

    private static void printFit(
            String wheel, List<Double> velocitySamples, List<Double> voltageSamples) {
        int count = 0;
        double sumX = 0.0;
        double sumY = 0.0;
        double sumXY = 0.0;
        double sumX2 = 0.0;
        for (int i = 0; i < Math.min(velocitySamples.size(), voltageSamples.size()); i++) {
            double x = velocitySamples.get(i);
            double y = voltageSamples.get(i);
            if (!Double.isFinite(x) || !Double.isFinite(y)) {
                continue;
            }
            count++;
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumX2 += x * x;
        }

        if (count < 10) {
            System.out.println(
                    "Shooter "
                            + wheel
                            + " kS/kV characterization failed: only "
                            + count
                            + " valid samples");
            return;
        }

        double denominator = count * sumX2 - sumX * sumX;
        if (Math.abs(denominator) < 1e-9) {
            System.out.println(
                    "Shooter " + wheel + " kS/kV characterization failed: no velocity range");
            return;
        }

        double kS = (sumY * sumX2 - sumX * sumXY) / denominator;
        double kV = (count * sumXY - sumX * sumY) / denominator;
        if (!Double.isFinite(kS) || !Double.isFinite(kV)) {
            System.out.println("Shooter " + wheel + " kS/kV characterization failed: invalid fit");
            return;
        }
        Logger.recordOutput("Shooter/Characterization/" + wheel + "/kS", kS);
        Logger.recordOutput("Shooter/Characterization/" + wheel + "/kV", kV);
        Logger.recordOutput("Shooter/Characterization/" + wheel + "/SampleCount", count);
        DecimalFormat format = new DecimalFormat("0.00000");
        System.out.println(
                "Shooter "
                        + wheel
                        + " characterization: kS="
                        + format.format(kS)
                        + ", kV="
                        + format.format(kV)
                        + " (V per rotations/s), samples="
                        + count);
    }
}
