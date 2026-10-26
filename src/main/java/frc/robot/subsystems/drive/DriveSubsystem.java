package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
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
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
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
    public static final double DRIVE_BASE_RADIUS = kDriveBaseRadiusMeters;

    private static final String[] SYSID_MODULE_NAMES = {
        "FrontLeft", "FrontRight", "BackLeft", "BackRight"
    };

    /** Number of modules aggregated for the SysId log motor (the drivetrain has four). */
    private static final int SYSID_MAX_MODULES = 4;

    /**
     * Volts-per-second ramp for the quasistatic SysId test. Slower (0.5 V/s) yields a cleaner kS/kV
     * fit; the WPILib default of 1 V/s is too fast for a 95 lb drivetrain and saturates too quickly
     * on a competition carpet.
     */
    private static final double SYSID_RAMP_VOLTS_PER_SEC = 0.5;

    /** Step voltage used for the dynamic SysId test. */
    private static final double SYSID_STEP_VOLTS = 6.0;

    /** Hard timeout for every SysId test so a runaway test can never exceed the field. */
    private static final double SYSID_TIMEOUT_SECONDS = 10.0;

    /**
     * Time to hold the drivetrain at zero voltage with the modules pointed forward before a SysId
     * test ramps. {@link com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveTranslation} commands
     * every module to 0 rad, which can take a few hundred milliseconds from a random power-on
     * angle. Without this dwell, the ramp applies voltage while the wheels are still slewing and
     * the robot lurches sideways.
     */
    private static final double SYSID_STEER_SETTLE_SECONDS = 1.0;

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
    private volatile SwerveSetpoint prevSetpoint = buildZeroedSetpoint(getModuleConstants().length);

    // The SwerveSetpointGenerator already desaturates wheel speeds internally, so we do not ask
    // the request to desaturate again (avoids redundant clipping of the generated setpoint).
    private final GeneratedSetpointRequest generatedSetpointRequest =
            new GeneratedSetpointRequest(getModuleConstants().length);

    public DriveSubsystem(DriveIO io, RobotState robotState) {
        this.io = io;
        this.robotState = robotState;

        configurePathPlanner();

        sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                Volts.of(SYSID_RAMP_VOLTS_PER_SEC).per(Seconds),
                                Volts.of(SYSID_STEP_VOLTS),
                                Seconds.of(SYSID_TIMEOUT_SECONDS),
                                state -> {
                                    // SignalLogger produces the .hoot files the SysId tool reads;
                                    // only run it while a test is active so we don't fill the disk
                                    // with unrelated data.
                                    if (state == SysIdRoutineLog.State.kNone) {
                                        SignalLogger.stop();
                                    } else {
                                        SignalLogger.start();
                                    }
                                    Logger.recordOutput("Drive/SysId/State", state.toString());
                                }),
                        new SysIdRoutine.Mechanism(
                                voltage -> io.setDriveOpenLoopVoltage(voltage),
                                log -> {
                                    // Log the drive motors under the SysId-standard keys so the AK
                                    // .wpilog can also be analyzed by SysId (in addition to the
                                    // CTRE .hoot log).
                                    SwerveModuleState[] states = inputs.moduleStates;
                                    double wheelRadius = getWheelRadiusMeters();
                                    double position = 0.0;
                                    double velocity = 0.0;
                                    double appliedVolts = 0.0;
                                    int n =
                                            Math.min(
                                                    Math.min(SYSID_MAX_MODULES, states.length),
                                                    Math.min(
                                                            inputs.modulePositions.length,
                                                            inputs.moduleDriveAppliedVolts.length));
                                    if (n > 0) {
                                        for (int i = 0; i < n; i++) {
                                            position +=
                                                    inputs.modulePositions[i].distanceMeters
                                                            / wheelRadius;
                                            velocity +=
                                                    states[i].speedMetersPerSecond / wheelRadius;
                                            appliedVolts += inputs.moduleDriveAppliedVolts[i];
                                        }
                                        log.motor("drive")
                                                .voltage(Volts.of(appliedVolts / n))
                                                .angularPosition(Radians.of(position / n))
                                                .angularVelocity(RadiansPerSecond.of(velocity / n));
                                    }
                                },
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

    /** Stop the wheels while keeping their last generated azimuth targets. */
    private static SwerveSetpoint buildStoppedSetpoint(SwerveSetpoint previous) {
        SwerveModuleState[] states = new SwerveModuleState[previous.moduleStates().length];
        for (int i = 0; i < states.length; i++) {
            states[i] = new SwerveModuleState(0.0, previous.moduleStates()[i].angle);
        }
        return new SwerveSetpoint(
                new ChassisSpeeds(), states, DriveFeedforwards.zeros(states.length));
    }

    /**
     * Passes the generator's per-module states directly into Phoenix. Reconstructing them from
     * chassis speeds loses the previous azimuth choice when a module is stopped and can undo the
     * generator's steering-rate limit near zero speed.
     */
    private static final class GeneratedSetpointRequest implements SwerveRequest {
        private final SwerveModule.ModuleRequest[] moduleRequests;
        private volatile SwerveSetpoint setpoint;

        private GeneratedSetpointRequest(int moduleCount) {
            moduleRequests = new SwerveModule.ModuleRequest[moduleCount];
            for (int i = 0; i < moduleCount; i++) {
                moduleRequests[i] =
                        new SwerveModule.ModuleRequest()
                                .withDriveRequest(DriveRequestType.Velocity)
                                .withSteerRequest(SwerveModule.SteerRequestType.Position);
            }
            setpoint = buildZeroedSetpoint(moduleCount);
        }

        private void setSetpoint(SwerveSetpoint newSetpoint) {
            if (newSetpoint.moduleStates().length != moduleRequests.length
                    || newSetpoint.feedforwards().robotRelativeForcesXNewtons().length
                            != moduleRequests.length
                    || newSetpoint.feedforwards().robotRelativeForcesYNewtons().length
                            != moduleRequests.length) {
                throw new IllegalArgumentException(
                        "Generated swerve setpoint must have one state and force pair per module");
            }
            setpoint = newSetpoint;
        }

        @Override
        public StatusCode apply(
                SwerveDrivetrain.SwerveControlParameters parameters,
                SwerveModule<?, ?, ?>... modulesToApply) {
            SwerveSetpoint currentSetpoint = setpoint;
            SwerveModuleState[] states = currentSetpoint.moduleStates();
            double[] forceX = currentSetpoint.feedforwards().robotRelativeForcesXNewtons();
            double[] forceY = currentSetpoint.feedforwards().robotRelativeForcesYNewtons();

            for (int i = 0; i < moduleRequests.length; i++) {
                moduleRequests[i]
                        .withState(
                                new SwerveModuleState(
                                        states[i].speedMetersPerSecond, states[i].angle))
                        .withUpdatePeriod(parameters.updatePeriod)
                        .withWheelForceFeedforwardX(forceX[i])
                        .withWheelForceFeedforwardY(forceY[i]);
                modulesToApply[i].apply(moduleRequests[i]);
            }
            return StatusCode.OK;
        }
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
        int expected = getModuleConstants().length;
        if (measured == null || measured.length != expected || speeds == null) {
            return prevSetpoint;
        }
        SwerveModuleState[] measuredSnapshot = new SwerveModuleState[measured.length];
        for (int i = 0; i < measured.length; i++) {
            measuredSnapshot[i] =
                    new SwerveModuleState(measured[i].speedMetersPerSecond, measured[i].angle);
        }
        return new SwerveSetpoint(
                new ChassisSpeeds(
                        speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond),
                measuredSnapshot,
                DriveFeedforwards.zeros(measured.length));
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
        public synchronized void accept(PathPlannerTrajectory t) {
            // A new trajectory may follow manual driving, pathfinding, or a previous path that
            // ended without stopping. Seed once at the handoff, then chain generated setpoints
            // while that trajectory remains active.
            if (t != null) {
                prevSetpoint = getCurrentSetpoint();
            }
            trajectory = t;
            timer.reset();
            timer.start();
            controller.reset(null, null);
        }

        @Override
        public synchronized void run() {
            if (!hasSetPriority) {
                hasSetPriority = Threads.setCurrentThreadPriority(true, 41);
            }

            PathPlannerTrajectory traj = trajectory;

            if (traj == null) return;

            if (traj.isStayStoppedTrajectory()) {
                SwerveSetpoint stopped = buildStoppedSetpoint(prevSetpoint);
                generatedSetpointRequest.setSetpoint(stopped);
                setControl(generatedSetpointRequest);
                prevSetpoint = stopped;
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

            Logger.recordOutput("PathPlanner/SetpointSpeeds", generated.robotRelativeSpeeds());
            Logger.recordOutput("PathPlanner/SetpointStates", generated.moduleStates());
            generatedSetpointRequest.setSetpoint(generated);
            setControl(generatedSetpointRequest);
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
                        getModuleConstants()[0].WheelRadius,
                        kDriveMaxSpeed,
                        kWheelCoefficientOfFriction,
                        DCMotor.getKrakenX60Foc(1),
                        getModuleConstants()[0].DriveMotorGearRatio,
                        getModuleConstants()[0].SlipCurrent,
                        1);

        var moduleConstants = getModuleConstants();
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
                    telemetry.setActivePath(activePath);
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

    /**
     * Returns a command to run a quasistatic SysId test in the requested direction.
     *
     * <p>The initial zero-voltage, modules-forward dwell ensures all four wheels are aligned and
     * the drivetrain is stationary before the ramp begins.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> io.setDriveOpenLoopVoltage(Volts.zero()))
                .withTimeout(SYSID_STEER_SETTLE_SECONDS)
                .andThen(sysId.quasistatic(direction))
                .withName("Drive SysId Quasistatic");
    }

    /** Returns a command to run a dynamic SysId test in the requested direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> io.setDriveOpenLoopVoltage(Volts.zero()))
                .withTimeout(SYSID_STEER_SETTLE_SECONDS)
                .andThen(sysId.dynamic(direction))
                .withName("Drive SysId Dynamic");
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
        io.setDriveOpenLoopVoltage(Volts.of(volts));
    }

    /** Run robot-relative chassis speed control (used for wheel radius characterization). */
    public void runVelocity(ChassisSpeeds speeds) {
        io.setDriveVelocity(speeds);
    }

    /** Get the gyro rotation (used for characterization). */
    public Rotation2d getRotation() {
        return io.getRotation();
    }

    /**
     * @return The nominal wheel radius configured in Tuner X, in meters.
     */
    public double getWheelRadiusMeters() {
        return getModuleConstants()[0].WheelRadius;
    }

    /**
     * @return The distance from the robot center to the furthest swerve module, in meters.
     */
    public double getDriveBaseRadiusMeters() {
        return DRIVE_BASE_RADIUS;
    }

    /** Get each module's drive-axle angle in radians (used for wheel radius characterization). */
    public double[] getWheelRadiusCharacterizationPositions() {
        return io.getWheelRadiusCharacterizationPositions();
    }

    /** Returns measured azimuth velocities for wheel-radius characterization settling. */
    public double[] getModuleSteerVelocitiesRadPerSec() {
        return io.getModuleSteerVelocitiesRadPerSec();
    }

    /** Get the average wheel velocity in rad/s (used for feedforward characterization). */
    public double getFFCharacterizationVelocity() {
        return io.getFFCharacterizationVelocity();
    }

    /** Average measured drive motor voltage, aligned with the latest measured module speeds. */
    public double getAverageDriveAppliedVoltage() {
        double[] voltages = inputs.moduleDriveAppliedVolts;
        if (voltages.length == 0) {
            return 0.0;
        }
        double sum = 0.0;
        for (double voltage : voltages) {
            sum += voltage;
        }
        return sum / voltages.length;
    }
}
