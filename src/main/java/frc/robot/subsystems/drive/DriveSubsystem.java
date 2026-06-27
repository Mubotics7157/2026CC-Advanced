package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.team254.lib.pathplanner.auto.AutoBuilder;
import com.team254.lib.pathplanner.config.ModuleConfig;
import com.team254.lib.pathplanner.config.PIDConstants;
import com.team254.lib.pathplanner.config.RobotConfig;
import com.team254.lib.pathplanner.controllers.PPHolonomicDriveController;
import com.team254.lib.pathplanner.controllers.PathFollowingController;
import com.team254.lib.pathplanner.trajectory.PathPlannerTrajectory;
import com.team254.lib.pathplanner.trajectory.PathPlannerTrajectoryState;
import com.team254.lib.pathplanner.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.lib.time.RobotTime;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;
import frc.robot.utils.simulations.MapleSimSwerveDrivetrain;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code DriveSubsystem} controls the robot's swerve drive base. It handles all driving
 * operations, including manual control, autonomous path following, and integration with sensors and
 * vision systems.
 */
public class DriveSubsystem extends SubsystemBase {
    private final DriveIO io;

    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    private final DriveViz telemetry = new DriveViz(kDriveMaxSpeed);

    private final RobotState robotState;

    private Controller controller;

    private final ApplyRobotSpeeds stopRequest =
            new ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final ApplyRobotSpeeds pathplannerAutoRequest =
            new ApplyRobotSpeeds()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withDesaturateWheelSpeeds(true);

    public DriveSubsystem(DriveIO io, RobotState robotState) {
        this.io = io;
        this.robotState = robotState;

        configurePathPlanner();
    }

    private ChassisSpeeds applyDeadband(ChassisSpeeds input) {
        if (Math.hypot(input.vxMetersPerSecond, input.vyMetersPerSecond) < 0.05) {
            input.vxMetersPerSecond = input.vyMetersPerSecond = 0.0;
        }
        if (Math.abs(input.omegaRadiansPerSecond) < 0.05) {
            input.omegaRadiansPerSecond = 0.0;
        }
        return input;
    }

    private class Controller implements Consumer<PathPlannerTrajectory>, Runnable {
        private final PathFollowingController controller;
        private volatile PathPlannerTrajectory trajectory = null;
        private volatile Timer timer = null;
        private Notifier notifier;

        boolean hasSetPriority = false;

        public Controller(PathFollowingController controller) {
            this.controller = controller;
            this.timer = new Timer();
            notifier = new Notifier(this);
            notifier.startPeriodic(0.01);
        }

        @Override
        public void accept(PathPlannerTrajectory t) {
            trajectory = t;
            timer.reset();
            timer.start();
            controller.reset(null, null);
        }

        @Override
        public void run() {
            if (!hasSetPriority) {
                hasSetPriority = Threads.setCurrentThreadPriority(true, 41);
            }

            PathPlannerTrajectory traj = trajectory;

            if (traj == null) return;

            if (traj.isStayStoppedTrajectory()) {
                setControl(stopRequest);
                trajectory = null;
                return;
            }

            double now = timer.get();
            PathPlannerTrajectoryState targetState = traj.sample(now);

            ChassisSpeeds speeds =
                    controller.calculateRobotRelativeSpeeds(
                            robotState.getLatestFieldToRobot().getValue(), targetState);
            if (DriverStation.isEnabled()) {
                setControl(
                        pathplannerAutoRequest
                                .withSpeeds(applyDeadband(speeds))
                                .withWheelForceFeedforwardsX(
                                        targetState.feedforwards.robotRelativeForcesXNewtons())
                                .withWheelForceFeedforwardsY(
                                        targetState.feedforwards.robotRelativeForcesYNewtons()));
            }
        }
    }

    @Override
    public void periodic() {
        double startTime = RobotTime.getTimestampSeconds();
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);

        telemetry.telemeterize(inputs.pose, inputs.odometryPeriod, inputs.moduleStates);

        // Mechanical Advantage-style logs
        Logger.recordOutput("Odometry/Robot", inputs.pose);
        Logger.recordOutput("SwerveStates/Measured", inputs.moduleStates);
        Logger.recordOutput(
                "SwerveChassisSpeeds/Measured", robotState.getLatestRobotRelativeChassisSpeed());

        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveChassisSpeeds/Setpoints", new ChassisSpeeds());
        } else {
            Logger.recordOutput("SwerveStates/Setpoints", inputs.moduleTargets);
            Logger.recordOutput(
                    "SwerveChassisSpeeds/Setpoints",
                    robotState.getLatestDesiredRobotRelativeChassisSpeeds());
        }

        robotState.incrementIterationCount();
        if (DriverStation.isDisabled()) {
            configureStandardDevsForDisabled();
        } else {
            configureStandardDevsForEnabled();
        }
        Logger.recordOutput(
                "Drive/latencyPeriodicSec", RobotTime.getTimestampSeconds() - startTime);
        Logger.recordOutput(
                "Drive/currentCommand",
                (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
    }

    private void configurePathPlanner() {
        ModuleConfig moduleConfig =
                new ModuleConfig(
                        kDrivetrain.getModuleConstants()[0].WheelRadius,
                        kDriveMaxSpeed,
                        kWheelCoefficientOfFriction,
                        DCMotor.getKrakenX60Foc(1),
                        kDrivetrain.getModuleConstants()[0].DriveMotorGearRatio,
                        kDrivetrain.getModuleConstants()[0].SlipCurrent,
                        1);

        var moduleConstants = kDrivetrain.getModuleConstants();
        RobotConfig robotConfig =
                new RobotConfig(
                        Constants.kRobotMassKg,
                        Constants.kRobotMomentOfInertia,
                        moduleConfig,
                        Constants.kCOGHeightMeters,
                        new Translation2d(
                                moduleConstants[0].LocationX, moduleConstants[0].LocationY),
                        new Translation2d(
                                moduleConstants[1].LocationX, moduleConstants[1].LocationY),
                        new Translation2d(
                                moduleConstants[2].LocationX, moduleConstants[2].LocationY),
                        new Translation2d(
                                moduleConstants[3].LocationX, moduleConstants[3].LocationY));

        controller =
                new Controller(
                        new PPHolonomicDriveController(
                                new PIDConstants(Constants.AutoConstants.kPLTEController, 0.0, 0.0),
                                new PIDConstants(Constants.AutoConstants.kPCTEController, 0.0, 0.0),
                                new PIDConstants(
                                        Constants.AutoConstants.kPThetaController, 0.0, 0.0),
                                0.01));

        AutoBuilder.configure(
                () -> robotState.getLatestFieldToRobot().getValue(),
                (pose) -> {},
                () -> robotState.getLatestFusedRobotRelativeChassisSpeed(),
                controller,
                robotConfig,
                () -> robotState.isRedAlliance(),
                this);

        PathPlannerLogging.setLogTargetPoseCallback(
                (pose) -> {
                    robotState.setTrajectoryTargetPose(pose);
                    Logger.recordOutput("PathPlanner/targetPose", pose);
                    Logger.recordOutput("Odometry/TrajectorySetpoint", pose);
                });

        PathPlannerLogging.setLogCurrentPoseCallback(
                (pose) -> {
                    robotState.setTrajectoryCurrentPose(pose);
                    Logger.recordOutput("PathPlanner/currentPose", pose);
                });

        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
                    Logger.recordOutput(
                            "PathPlanner/activePath", activePath.toArray(new Pose2d[0]));
                });

        PathPlannerLogging.setLogTargetChassisSpeedsCallback(
                (chassisSpeeds) -> {
                    Logger.recordOutput("PathPlanner/targetChassisSpeeds", chassisSpeeds);
                    Logger.recordOutput("SwerveChassisSpeeds/TrajectorySetpoint", chassisSpeeds);
                });
    }

    public void resetOdometry(Pose2d pose) {
        io.resetOdometry(pose);
    }

    public Consumer<PathPlannerTrajectory> getController() {
        return controller;
    }

    public void setControl(SwerveRequest request) {
        io.setControl(request);
    }

    // API
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return io.applyRequest(requestSupplier, this).withName("Swerve drive request");
    }

    public void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate) {
        io.addVisionMeasurement(visionFieldPoseEstimate);
    }

    public void setStateStdDevs(double xStd, double yStd, double rotStd) {
        io.setStateStdDevs(xStd, yStd, rotStd);
    }

    public void configureStandardDevsForDisabled() {
        setStateStdDevs(kDisabledDriveXStdDev, kDisabledDriveYStdDev, kDisabledDriveRotStdDev);
    }

    public void configureStandardDevsForEnabled() {
        setStateStdDevs(kEnabledDriveXStdDev, kEnabledDriveYStdDev, kEnabledDriveRotStdDev);
    }

    public MapleSimSwerveDrivetrain getMapleSimDrive() {
        if (io instanceof DriveIOSim) {
            return ((DriveIOSim) io).getMapleSimDrive();
        }
        return null;
    }
}
