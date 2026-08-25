package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

/** Solves fixed-shooter aim heading and wheel velocity, including launch-on-the-move lookahead. */
public final class ShooterAiming {

    private ShooterAiming() {}

    public record ShotSolution(
            Translation2d target,
            Translation2d currentShooterPosition,
            Translation2d compensatedShooterPosition,
            Rotation2d robotHeading,
            double distanceMeters,
            double timeOfFlightSeconds,
            double shooterVelocityRadPerSec,
            double hoodAngleRad,
            double headingErrorRad,
            boolean inRange,
            boolean headingReady) {

        public boolean readyToFeed() {
            return inRange && headingReady;
        }
    }

    public static ShotSolution calculate(RobotState robotState) {
        Pose2d robotPose =
                robotState.getPredictedFieldToRobot(ShooterSetpoints.PRELAUNCH_LOOKAHEAD_SECONDS);
        ChassisSpeeds robotRelativeSpeeds = robotState.getLatestRobotRelativeChassisSpeed();
        Translation2d target = ShooterSetpoints.getTargetTranslation(robotState.isRedAlliance());

        Translation2d shooterStart =
                robotPose
                        .getTranslation()
                        .plus(
                                ShooterSetpoints.ROBOT_TO_SHOOTER_EXIT.rotateBy(
                                        robotPose.getRotation()));
        Translation2d shooterVelocity =
                calculateShooterFieldVelocity(
                        ShooterSetpoints.ROBOT_TO_SHOOTER_EXIT, robotPose, robotRelativeSpeeds);

        Translation2d compensatedShooter = shooterStart;
        double distance = get3dShotDistance(compensatedShooter, target);
        double timeOfFlight = ShooterSetpoints.getTimeOfFlightSeconds(distance);

        for (int i = 0; i < ShooterSetpoints.LAUNCH_LOOKAHEAD_ITERATIONS; i++) {
            compensatedShooter = shooterStart.plus(shooterVelocity.times(timeOfFlight));
            distance = get3dShotDistance(compensatedShooter, target);
            timeOfFlight = ShooterSetpoints.getTimeOfFlightSeconds(distance);
        }

        Translation2d shooterToTarget = target.minus(compensatedShooter);
        Rotation2d fieldShotAngle = new Rotation2d(shooterToTarget.getX(), shooterToTarget.getY());
        Rotation2d robotHeading =
                fieldShotAngle.minus(
                        new Rotation2d(ShooterSetpoints.ROBOT_SHOOTING_YAW_OFFSET_RAD));

        double headingError =
                MathUtil.angleModulus(
                        robotHeading.getRadians() - robotPose.getRotation().getRadians());
        boolean headingReady =
                Math.abs(headingError) < ShooterSetpoints.HEADING_TOLERANCE_RAD
                        && Math.abs(robotRelativeSpeeds.omegaRadiansPerSecond)
                                < ShooterSetpoints.OMEGA_TOLERANCE_RAD_PER_SEC;
        boolean inRange = ShooterSetpoints.isInRange(distance);

        ShotSolution solution =
                new ShotSolution(
                        target,
                        shooterStart,
                        compensatedShooter,
                        robotHeading,
                        distance,
                        timeOfFlight,
                        ShooterSetpoints.getVelocityRadPerSec(distance),
                        ShooterSetpoints.getHoodAngleRad(distance),
                        headingError,
                        inRange,
                        headingReady);
        log(solution);
        return solution;
    }

    private static Translation2d calculateShooterFieldVelocity(
            Translation2d shooterOffsetRobotFrame,
            Pose2d robotPose,
            ChassisSpeeds robotRelativeSpeeds) {
        double pointVxRobot =
                robotRelativeSpeeds.vxMetersPerSecond
                        - robotRelativeSpeeds.omegaRadiansPerSecond
                                * shooterOffsetRobotFrame.getY();
        double pointVyRobot =
                robotRelativeSpeeds.vyMetersPerSecond
                        + robotRelativeSpeeds.omegaRadiansPerSecond
                                * shooterOffsetRobotFrame.getX();

        return new Translation2d(pointVxRobot, pointVyRobot).rotateBy(robotPose.getRotation());
    }

    private static double get3dShotDistance(Translation2d shooterPosition, Translation2d target) {
        double horizontalDistance = shooterPosition.getDistance(target);
        double heightDelta =
                ShooterSetpoints.TARGET_HEIGHT_METERS - ShooterSetpoints.SHOOTER_EXIT_HEIGHT_METERS;
        return Math.hypot(horizontalDistance, heightDelta);
    }

    private static void log(ShotSolution solution) {
        Logger.recordOutput("Shooter/Aiming/Target", solution.target());
        Logger.recordOutput(
                "Shooter/Aiming/CurrentShooterPosition", solution.currentShooterPosition());
        Logger.recordOutput(
                "Shooter/Aiming/CompensatedShooterPosition", solution.compensatedShooterPosition());
        Logger.recordOutput("Shooter/Aiming/RobotHeading", solution.robotHeading());
        Logger.recordOutput("Shooter/Aiming/DistanceMeters", solution.distanceMeters());
        Logger.recordOutput("Shooter/Aiming/TimeOfFlightSeconds", solution.timeOfFlightSeconds());
        Logger.recordOutput(
                "Shooter/Aiming/ShooterVelocityRadPerSec", solution.shooterVelocityRadPerSec());
        Logger.recordOutput("Shooter/Aiming/HoodAngleRad", solution.hoodAngleRad());
        Logger.recordOutput("Shooter/Aiming/HoodAngleDeg", Math.toDegrees(solution.hoodAngleRad()));
        Logger.recordOutput("Shooter/Aiming/HeadingErrorRad", solution.headingErrorRad());
        Logger.recordOutput("Shooter/Aiming/InRange", solution.inRange());
        Logger.recordOutput("Shooter/Aiming/HeadingReady", solution.headingReady());
        Logger.recordOutput("Shooter/Aiming/ReadyToFeed", solution.readyToFeed());
    }
}
