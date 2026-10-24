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
import com.team254.lib.pathplanner.util.DriveFeedforwards;
import com.team254.lib.pathplanner.util.PathPlannerLogging;
import com.team254.lib.pathplanner.util.swerve.SwerveSetpoint;
import com.team254.lib.pathplanner.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
    /** Drive base radius in meters, calculated from the module positions (furthest module). */
    public static final double DRIVE_BASE_RADIUS =
            Math.max(
                    Math.max(
                            Math.hypot(
                                    CompTunerConstants.FrontLeft.LocationX,
                                    CompTunerConstants.FrontLeft.LocationY),
                            Math.hypot(
                                    CompTunerConstants.FrontRight.LocationX,
                                    CompTunerConstants.FrontRight.LocationY)),
                    Math.max(
                            Math.hypot(
                                    CompTunerConstants.BackLeft.LocationX,
                                    CompTunerConstants.BackLeft.LocationY),
                            Math.hypot(
                                    CompTunerConstants.BackRight.LocationX,
                                    CompTunerConstants.BackRight.LocationY)));

    private static final String[] SYSID_MODULE_NAMES = {
        "FrontLeft", "FrontRight", "BackLeft", "BackRight"
    };

    private final DriveIO io;

    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    private final DriveViz telemetry = new DriveViz(kDriveMaxSpeed);

    private final RobotState robotState;

    private final SysIdRoutine sysId;

    private Controller controller;

    /** Robot configuration used for path generation and the swerve setpoint generator. */
    private RobotConfig robotConfig;

    /**
     * Generates module setpoints from chassis speeds while respecting the robot's kinematic,
     * friction, and torque limits. Used to smooth path-following outputs before they hit the
     * drivetrain.
     */
    private SwerveSetpointGenerator swerveSetpointGenerator;

            /**
     * The most recent generated setpoint, fed back in as the "previous" state each loop. Seeded
     * from measured state at startup and reseeded on stop.
     */
    private volatile SwerveSetpoint prevSetpoint =
            buildZeroedSetpoint(kDrivetrain.getModuleConstants().length);

    private final ApplyRobotSpeeds stopRequest =
            new ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // The SwerveSetpointGenerator already desaturates wheel speeds internally, so we do not ask
    // the request to desaturate again (avoids redundant clipping of the generated setpoint).
    private final ApplyRobotSpeeds pathplannerAutoRequest =
            new ApplyRobotSpeeds()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withDesaturateWheelSpeeds(false);

    public DriveSubsystem(DriveIO io, RobotState robotState) {
        this.io = io;
        this.robotState = robotState;

        configurePathPlanner();

        sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                state -> {
                                    Logger.recordOutput("SysIdTestState", state.toString());
                                    Logger.recordOutput("Drive/SysIdState", state.toString());
                                }),
                        new SysIdRoutine.Mechanism(
                                voltage ->
                                        io.runCharacterization(
                                                voltage.in(edu.wpi.first.units.Units.Volts)),
                                null,
                                this));
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

    /** Build an all-zero setpoint appropriate for seeding the {@link SwerveSetpointGenerator}. */
    private static SwerveSetpoint buildZeroedSetpoint(int numModules) {
        SwerveModuleState[] states = new SwerveModuleState[numModules];
        for (int i = 0; i < numModules; i++) {
            states[i] = new SwerveModuleState();
        }
        return new SwerveSetpoint(new ChassisSpeeds(), states, DriveFeedforwards.zeros(numModules));
    }

            /**
     * Snapshot the current measured drivetrain state as a {@link SwerveSetpoint}, used to seed the
     * generator's "previous" state at startup and whenever a trajectory stops. Falls back to the
     * cached {@link #prevSetpoint} if measured states aren't available yet (e.g. at startup or in
     * replay of empty logs).
     */
    private SwerveSetpoint getCurrentSetpoint() {
        SwerveModuleState[] measured = inputs.moduleStates;
        ChassisSpeeds speeds = robotState.getLatestRobotRelativeChassisSpeed();
        int expected = kDrivetrain.getModuleConstants().length;
        if (measured == null || measured.length != expected || speeds == null) {
            return prevSetpoint;
        }
        return new SwerveSetpoint(speeds, measured, DriveFeedforwards.zeros(measured.length));
    }

    private class Controller implements Consumer<PathPlannerTrajectory>, Runnable {
        /** The Controller's background notifier loop period, in seconds. */
        private static final double LOOP_PERIOD = 0.01;

        private final PathFollowingController controller;
        private volatile PathPlannerTrajectory trajectory = null;
        private volatile Timer timer = null;
        private Notifier notifier;

        boolean hasSetPriority = false;

        public Controller(PathFollowingController controller) {
            this.controller = controller;
            this.timer = new Timer();
            notifier = new Notifier(this);
            notifier.startPeriodic(LOOP_PERIOD);
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
                // Reseed the generator's "previous" state from reality so the next trajectory
                // ramps from where the robot actually is instead of a stale setpoint.
                prevSetpoint = getCurrentSetpoint();
                trajectory = null;
                return;
            }

            double now = timer.get();
            PathPlannerTrajectoryState targetState = traj.sample(now);

            ChassisSpeeds desiredSpeeds =
                    applyDeadband(
                            controller.calculateRobotRelativeSpeeds(
                                    robotState.getLatestFieldToRobot().getValue(), targetState));

            if (!DriverStation.isEnabled()) {
                return;
            }

                        // Run the controller's desired chassis speeds through the swerve setpoint generator.
            // This enforces the robot's kinematic, friction, and torque limits so the drivetrain
            // accelerates/rotates as fast as physics allows without slipping the wheels, and keeps
            // module rotations within their steer velocity limit.
            //
            // Per the PathPlanner docs, chain off the previous *generated* setpoint rather than
            // measured state. The generator rate-limits setpoint-to-setpoint; feeding it noisy
            // measured state each loop would defeat the smoothing and add jitter. (prevSetpoint is
            // seeded from measured state at startup and on stop.)
            SwerveSetpoint generated =
                    swerveSetpointGenerator.generateSetpoint(
                            prevSetpoint, desiredSpeeds, LOOP_PERIOD);
            prevSetpoint = generated;

            // Prefer the setpoint generator's own feedforwards (they account for the torque and
            // force limits it applied); fall back to the trajectory feedforwards only if the
            // generator produced none.
            DriveFeedforwards ff = generated.feedforwards();

            Logger.recordOutput("PathPlanner/SetpointSpeeds", generated.robotRelativeSpeeds());
            Logger.recordOutput("PathPlanner/SetpointStates", generated.moduleStates());

            setControl(
                    pathplannerAutoRequest
                            .withSpeeds(generated.robotRelativeSpeeds())
                            .withWheelForceFeedforwardsX(ff.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(ff.robotRelativeForcesYNewtons()));
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
        logRobotCorners(inputs.pose);
        Logger.recordOutput("SwerveStates/Measured", inputs.moduleStates);
        for (int i = 0;
                i < SYSID_MODULE_NAMES.length
                        && i < inputs.modulePositions.length
                        && i < inputs.moduleStates.length
                        && i < inputs.moduleDriveAppliedVolts.length;
                i++) {
            String key = "Drive/SysId/" + SYSID_MODULE_NAMES[i] + "/";
            Logger.recordOutput(key + "PositionMeters", inputs.modulePositions[i].distanceMeters);
            Logger.recordOutput(
                    key + "VelocityMetersPerSec", inputs.moduleStates[i].speedMetersPerSecond);
            Logger.recordOutput(key + "AppliedVolts", inputs.moduleDriveAppliedVolts[i]);
        }
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
        robotConfig =
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

        // Build the swerve setpoint generator using the robot's steer speed limit. The generator
        // produces module setpoints that respect acceleration, friction, and torque limits so that
        // path-following commands feel smooth and don't slip the wheels.
        swerveSetpointGenerator =
                new SwerveSetpointGenerator(robotConfig, kMaxSteerVelocityRadPerSec);

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

    /** Returns a command to run a quasistatic SysId test in the requested direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> io.runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic SysId test in the requested direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> io.runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.dynamic(direction));
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

    // --- Robot corner visualization (AdvantageScope) ---

        /** Pre-computed constant corner offsets (never change), in CCW order starting front-left. */
    private static final double HALF_LENGTH = Units.inchesToMeters(kBumperLengthInches / 2.0);

    private static final double HALF_WIDTH = Units.inchesToMeters(kBumperWidthInches / 2.0);

    private static final Translation2d[] CORNER_OFFSETS = {
        new Translation2d(HALF_LENGTH, HALF_WIDTH), // Front Left
        new Translation2d(HALF_LENGTH, -HALF_WIDTH), // Front Right
        new Translation2d(-HALF_LENGTH, -HALF_WIDTH), // Back Right
        new Translation2d(-HALF_LENGTH, HALF_WIDTH) // Back Left
    };

    /** Log the robot's bumper corners as Pose3d[] for 3D AdvantageScope visualization. */
    private void logRobotCorners(Pose2d robotPose) {
        Pose3d[] corners = new Pose3d[4];
        Rotation3d rotation = new Rotation3d(0, 0, robotPose.getRotation().getRadians());
        for (int i = 0; i < 4; i++) {
            Translation2d fieldOffset = CORNER_OFFSETS[i].rotateBy(robotPose.getRotation());
            corners[i] =
                    new Pose3d(
                            robotPose.getX() + fieldOffset.getX(),
                            robotPose.getY() + fieldOffset.getY(),
                            0,
                            rotation);
        }
        Logger.recordOutput("Odometry/RobotCorners", corners);
    }

    // --- Characterization helpers ---

    /** Run open-loop voltage control for SysId characterization. */
    public void runCharacterization(double volts) {
        io.runCharacterization(volts);
    }

    /** Run robot-relative chassis speed control (used for wheel radius characterization). */
    public void runVelocity(ChassisSpeeds speeds) {
        io.runVelocity(speeds);
    }

    /** Get the gyro rotation (used for characterization). */
    public Rotation2d getRotation() {
        return io.getRotation();
    }

    /** Get each module's wheel position in radians (used for wheel radius characterization). */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        double wheelRadius = CompTunerConstants.FrontLeft.WheelRadius;
        for (int i = 0; i < 4 && i < inputs.modulePositions.length; i++) {
            values[i] = inputs.modulePositions[i].distanceMeters / wheelRadius;
        }
        return values;
    }

    /** Get the average wheel velocity in rad/s (used for feedforward characterization). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        double wheelRadius = CompTunerConstants.FrontLeft.WheelRadius;
        for (int i = 0; i < 4 && i < inputs.moduleStates.length; i++) {
            output += inputs.moduleStates[i].speedMetersPerSecond / wheelRadius / 4.0;
        }
        return output;
    }
}
