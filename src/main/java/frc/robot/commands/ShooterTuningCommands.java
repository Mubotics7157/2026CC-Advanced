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
                            shooter.setVelocity(
                                    Units.rotationsPerMinuteToRadiansPerSecond(
                                            ShooterConstants.TUNING_VELOCITY_RPM.get()));
                            shooter.setHoodAngle(
                                    Units.degreesToRadians(
                                            ShooterConstants.TUNING_HOOD_ANGLE_DEG.get()));
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

    /** Ramps all three wheels up to a bounded voltage and prints per-wheel kS/kV fits. */
    public static Command feedforwardCharacterization(
            Shooter shooter, Superstructure superstructure) {
        Timer timer = new Timer();
        List<Double> leftVelocity = new ArrayList<>();
        List<Double> middleVelocity = new ArrayList<>();
        List<Double> rightVelocity = new ArrayList<>();
        List<Double> appliedVoltage = new ArrayList<>();

        Command rampAndSample =
                Commands.run(
                                () -> {
                                    double voltage =
                                            Math.min(
                                                    maxCharacterizationVolts(),
                                                    Math.max(
                                                            0.0,
                                                            timer.get()
                                                                    * Math.max(
                                                                            0.05,
                                                                            ShooterConstants
                                                                                    .CHARACTERIZATION_RAMP_VOLTS_PER_SEC
                                                                                    .get())));
                                    shooter.runVoltage(voltage);
                                    leftVelocity.add(Math.abs(shooter.getLeftVelocityRadPerSec()));
                                    middleVelocity.add(
                                            Math.abs(shooter.getMiddleVelocityRadPerSec()));
                                    rightVelocity.add(
                                            Math.abs(shooter.getRightVelocityRadPerSec()));
                                    appliedVoltage.add(voltage);
                                },
                                shooter,
                                superstructure)
                        .until(
                                () ->
                                        timer.get()
                                                >= maxCharacterizationVolts()
                                                        / Math.max(
                                                                0.05,
                                                                ShooterConstants
                                                                        .CHARACTERIZATION_RAMP_VOLTS_PER_SEC
                                                                        .get()));

        return Commands.sequence(
                        Commands.runOnce(
                                () -> {
                                    leftVelocity.clear();
                                    middleVelocity.clear();
                                    rightVelocity.clear();
                                    appliedVoltage.clear();
                                    superstructure.setShooterTuningOverride(true);
                                    superstructure.setGoal(Goal.IDLE);
                                    shooter.stop();
                                },
                                shooter,
                                superstructure),
                        Commands.runOnce(timer::restart),
                        rampAndSample)
                .finallyDo(
                        () -> {
                            shooter.stop();
                            superstructure.setShooterTuningOverride(false);
                            superstructure.setGoal(Goal.IDLE);
                            printFit("Left", leftVelocity, appliedVoltage);
                            printFit("Middle", middleVelocity, appliedVoltage);
                            printFit("Right", rightVelocity, appliedVoltage);
                        })
                .withName("Shooter Tuning: Feedforward Characterization");
    }

    private static double maxCharacterizationVolts() {
        return Math.min(
                HARD_MAX_CHARACTERIZATION_VOLTS,
                Math.max(0.0, ShooterConstants.CHARACTERIZATION_MAX_VOLTS.get()));
    }

    private static void printFit(
            String wheel, List<Double> velocitySamples, List<Double> voltageSamples) {
        int count = Math.min(velocitySamples.size(), voltageSamples.size());
        if (count < 10) {
            System.out.println(
                    "Shooter "
                            + wheel
                            + " kS/kV characterization failed: only "
                            + count
                            + " samples");
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
            System.out.println(
                    "Shooter " + wheel + " kS/kV characterization failed: no velocity range");
            return;
        }

        double kS = (sumY * sumX2 - sumX * sumXY) / denominator;
        double kV = (count * sumXY - sumX * sumY) / denominator;
        DecimalFormat format = new DecimalFormat("0.00000");
        System.out.println(
                "Shooter "
                        + wheel
                        + " characterization: kS="
                        + format.format(kS)
                        + ", kV="
                        + format.format(kV)
                        + " (V per rad/s), samples="
                        + count);
    }
}
