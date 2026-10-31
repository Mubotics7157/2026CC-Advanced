package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

/** Solves the stationary fixed-shooter aim heading and wheel velocity. */
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
            double hoodServoPosition,
            double headingErrorRad,
            double translationSpeedMetersPerSec,
            double omegaRadiansPerSec,
            boolean inRange,
            boolean headingReady) {

        public boolean readyToFeed() {
            return inRange && headingReady;
        }
    }

    public static ShotSolution calculate(RobotState robotState) {
        return calculate(
                robotState,
                ShooterSetpoints.getTargetTranslation(robotState.isRedAlliance()),
                false);
    }

    /** Calculates a full-field ferry solution aimed at the nearest corner on our alliance wall. */
    public static ShotSolution calculateAllianceCorner(RobotState robotState) {
        Translation2d robotPosition =
                robotState.getLatestFieldToRobot().getValue().getTranslation();
        Translation2d target =
                ShooterSetpoints.getAllianceCornerTranslation(
                        robotState.isRedAlliance(), robotPosition);
        return calculate(robotState, target, true);
    }

    private static ShotSolution calculate(
            RobotState robotState, Translation2d target, boolean allianceCornerFerry) {
        Pose2d robotPose = robotState.getLatestFieldToRobot().getValue();
        ChassisSpeeds robotRelativeSpeeds = robotState.getLatestRobotRelativeChassisSpeed();
        Translation2d robotPosition = robotPose.getTranslation();

        // The shooter exit is offset from the robot center. Solve for the final heading and exit
        // point together so the commanded heading points the shooter at the target.
        Rotation2d robotHeading =
                new Rotation2d(
                                target.getX() - robotPosition.getX(),
                                target.getY() - robotPosition.getY())
                        .minus(new Rotation2d(ShooterSetpoints.ROBOT_SHOOTING_YAW_OFFSET_RAD));
        Translation2d aimedShooterPosition = robotPosition;
        for (int i = 0; i < 5; i++) {
            aimedShooterPosition =
                    robotPosition.plus(
                            ShooterSetpoints.ROBOT_TO_SHOOTER_EXIT.rotateBy(robotHeading));
            Translation2d shooterToTarget = target.minus(aimedShooterPosition);
            robotHeading =
                    new Rotation2d(shooterToTarget.getX(), shooterToTarget.getY())
                            .minus(
                                    new Rotation2d(
                                            ShooterSetpoints.ROBOT_SHOOTING_YAW_OFFSET_RAD));
        }
        aimedShooterPosition =
                robotPosition.plus(ShooterSetpoints.ROBOT_TO_SHOOTER_EXIT.rotateBy(robotHeading));
        Translation2d currentShooterPosition =
                robotPosition.plus(
                        ShooterSetpoints.ROBOT_TO_SHOOTER_EXIT.rotateBy(robotPose.getRotation()));
        double distance = get3dShotDistance(aimedShooterPosition, target);
        double timeOfFlight = getTimeOfFlightSeconds(distance, allianceCornerFerry);
        double translationSpeed =
                Math.hypot(
                        robotRelativeSpeeds.vxMetersPerSecond,
                        robotRelativeSpeeds.vyMetersPerSecond);

        double headingError =
                MathUtil.angleModulus(
                        robotHeading.getRadians() - robotPose.getRotation().getRadians());
        boolean headingReady =
                Math.abs(headingError) < ShooterSetpoints.HEADING_TOLERANCE_RAD
                        && Math.abs(robotRelativeSpeeds.omegaRadiansPerSecond)
                                < ShooterSetpoints.OMEGA_TOLERANCE_RAD_PER_SEC
                        && translationSpeed
                                < ShooterSetpoints.TRANSLATION_TOLERANCE_METERS_PER_SEC;
        boolean inRange = allianceCornerFerry || ShooterSetpoints.isInRange(distance);
        double shooterVelocityRadPerSec =
                allianceCornerFerry
                        ? ShooterSetpoints.getAllianceCornerFerryVelocityRadPerSec()
                        : ShooterSetpoints.getVelocityRadPerSec(distance);
        double hoodAngleRad =
                allianceCornerFerry
                        ? ShooterConstants.HOOD_MAX_ANGLE_RAD
                        : ShooterSetpoints.getHoodAngleRad(distance);

        ShotSolution solution =
                new ShotSolution(
                        target,
                        currentShooterPosition,
                        aimedShooterPosition,
                        robotHeading,
                        distance,
                        timeOfFlight,
                        shooterVelocityRadPerSec,
                        hoodAngleRad,
                        ShooterConstants.hoodAngleToServoPosition(hoodAngleRad),
                        headingError,
                        translationSpeed,
                        robotRelativeSpeeds.omegaRadiansPerSecond,
                        inRange,
                        headingReady);
        log(solution);
        return solution;
    }

    private static double getTimeOfFlightSeconds(double distanceMeters, boolean ferry) {
        return ferry
                ? ShooterSetpoints.ALLIANCE_CORNER_FERRY_FLIGHT_TIME_SECONDS.get()
                : ShooterSetpoints.getTimeOfFlightSeconds(distanceMeters);
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
        Logger.recordOutput("Shooter/Aiming/HoodServoPosition", solution.hoodServoPosition());
        Logger.recordOutput("Shooter/Aiming/HeadingErrorRad", solution.headingErrorRad());
        Logger.recordOutput(
                "Shooter/Aiming/TranslationSpeedMetersPerSec",
                solution.translationSpeedMetersPerSec());
        Logger.recordOutput("Shooter/Aiming/OmegaRadiansPerSec", solution.omegaRadiansPerSec());
        Logger.recordOutput("Shooter/Aiming/InRange", solution.inRange());
        Logger.recordOutput("Shooter/Aiming/HeadingReady", solution.headingReady());
        Logger.recordOutput("Shooter/Aiming/ReadyToFeed", solution.readyToFeed());
    }
}
