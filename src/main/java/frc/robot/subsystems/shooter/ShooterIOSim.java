package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
    private static final double SIM_LOOP_PERIOD = 0.02;
    private static final double MAX_VOLTAGE = 12.0;

    private final DCMotorSim leftSim;
    private final DCMotorSim middleSim;
    private final DCMotorSim rightSim;

    private final double[] profiledVelocityRotationsPerSec = new double[3];
    private final double[] integralVelocityError = new double[3];
    private final double[] previousVelocityError = new double[3];

    private double leftAppliedVolts = 0.0;
    private double middleAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;
    private double velocitySetpoint = 0.0;
    private double hoodServoPosition = ShooterConstants.HOOD_SERVO_MIN_POSITION;

    public ShooterIOSim() {
        leftSim = createSim();
        middleSim = createSim();
        rightSim = createSim();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        leftSim.update(SIM_LOOP_PERIOD);
        middleSim.update(SIM_LOOP_PERIOD);
        rightSim.update(SIM_LOOP_PERIOD);

        inputs.leftPosition = leftSim.getAngularPositionRad();
        inputs.middlePosition = middleSim.getAngularPositionRad();
        inputs.rightPosition = rightSim.getAngularPositionRad();
        inputs.leftVelocity = leftSim.getAngularVelocityRadPerSec();
        inputs.middleVelocity = middleSim.getAngularVelocityRadPerSec();
        inputs.rightVelocity = rightSim.getAngularVelocityRadPerSec();

        inputs.leftVoltage = leftAppliedVolts;
        inputs.middleVoltage = middleAppliedVolts;
        inputs.rightVoltage = rightAppliedVolts;

        inputs.leftCurrent = leftSim.getCurrentDrawAmps();
        inputs.middleCurrent = middleSim.getCurrentDrawAmps();
        inputs.rightCurrent = rightSim.getCurrentDrawAmps();

        inputs.velocitySetpoint = velocitySetpoint;
        inputs.requestedVelocity = velocitySetpoint;
        inputs.leftHoodServoPosition = hoodServoPosition;
        inputs.rightHoodServoPosition = hoodServoPosition;
        inputs.hoodServoSetpoint = hoodServoPosition;
        inputs.hoodAtSetpoint = true;
        inputs.hoodHardwareConfigured = true;
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        velocitySetpoint = velocityRadPerSec;
        leftAppliedVolts = calculateVoltage(leftSim, ShooterConstants.Flywheel.LEFT, 0);
        middleAppliedVolts = calculateVoltage(middleSim, ShooterConstants.Flywheel.MIDDLE, 1);
        rightAppliedVolts = calculateVoltage(rightSim, ShooterConstants.Flywheel.RIGHT, 2);

        leftSim.setInputVoltage(leftAppliedVolts);
        middleSim.setInputVoltage(middleAppliedVolts);
        rightSim.setInputVoltage(rightAppliedVolts);
    }

    @Override
    public void runVoltage(double volts) {
        double clampedVolts = MathUtil.clamp(volts, -MAX_VOLTAGE, MAX_VOLTAGE);
        leftAppliedVolts = clampedVolts;
        middleAppliedVolts = clampedVolts;
        rightAppliedVolts = clampedVolts;
        leftSim.setInputVoltage(clampedVolts);
        middleSim.setInputVoltage(clampedVolts);
        rightSim.setInputVoltage(clampedVolts);
    }

    @Override
    public void setHoodServoPosition(double position) {
        hoodServoPosition =
                MathUtil.clamp(
                        position,
                        ShooterConstants.HOOD_SERVO_MIN_POSITION,
                        ShooterConstants.HOOD_SERVO_MAX_POSITION);
    }

    @Override
    public void stop() {
        velocitySetpoint = 0.0;
        leftAppliedVolts = 0.0;
        middleAppliedVolts = 0.0;
        rightAppliedVolts = 0.0;
        resetControllers();
        leftSim.setInputVoltage(0.0);
        middleSim.setInputVoltage(0.0);
        rightSim.setInputVoltage(0.0);
    }

    private double calculateVoltage(DCMotorSim sim, ShooterConstants.Flywheel flywheel, int index) {
        double targetRotationsPerSec = Units.radiansToRotations(velocitySetpoint);
        double previousProfileVelocity = profiledVelocityRotationsPerSec[index];
        double maxVelocityChange =
                ShooterConstants.getMotionMagicAccelerationRotationsPerSecSq() * SIM_LOOP_PERIOD;
        profiledVelocityRotationsPerSec[index] +=
                MathUtil.clamp(
                        targetRotationsPerSec - previousProfileVelocity,
                        -maxVelocityChange,
                        maxVelocityChange);
        double profileAccelerationRotationsPerSecSq =
                (profiledVelocityRotationsPerSec[index] - previousProfileVelocity)
                        / SIM_LOOP_PERIOD;

        double measuredRotationsPerSec =
                Units.radiansToRotations(sim.getAngularVelocityRadPerSec());
        double velocityError = profiledVelocityRotationsPerSec[index] - measuredRotationsPerSec;
        integralVelocityError[index] += velocityError * SIM_LOOP_PERIOD;
        double velocityErrorDerivative =
                (velocityError - previousVelocityError[index]) / SIM_LOOP_PERIOD;
        previousVelocityError[index] = velocityError;

        double requestedCurrentAmps =
                Math.copySign(
                                flywheel.kS.getClamped(0.0, 12.0),
                                profiledVelocityRotationsPerSec[index])
                        + flywheel.kV.getClamped(0.0, 12.0) * profiledVelocityRotationsPerSec[index]
                        + flywheel.kA.getClamped(0.0, 12.0) * profileAccelerationRotationsPerSecSq
                        + flywheel.kP.getClamped(0.0, 100.0) * velocityError
                        + flywheel.kI.getClamped(0.0, 100.0) * integralVelocityError[index]
                        + flywheel.kD.getClamped(0.0, 100.0) * velocityErrorDerivative;

        double maxCurrentAmps = ShooterConstants.getPeakTorqueCurrentAmps();
        if (maxCurrentAmps <= 0.0) {
            return 0.0;
        }
        double appliedCurrentAmps =
                MathUtil.clamp(requestedCurrentAmps, -maxCurrentAmps, maxCurrentAmps);
        double maxVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(ShooterConstants.getMaxVelocityRPM());
        double backEmfVolts =
                maxVelocityRadPerSec <= 0.0
                        ? 0.0
                        : sim.getAngularVelocityRadPerSec() / maxVelocityRadPerSec * MAX_VOLTAGE;
        return MathUtil.clamp(
                backEmfVolts + appliedCurrentAmps / maxCurrentAmps * MAX_VOLTAGE,
                -MAX_VOLTAGE,
                MAX_VOLTAGE);
    }

    private void resetControllers() {
        for (int i = 0; i < profiledVelocityRotationsPerSec.length; i++) {
            profiledVelocityRotationsPerSec[i] = 0.0;
            integralVelocityError[i] = 0.0;
            previousVelocityError[i] = 0.0;
        }
    }

    private static DCMotorSim createSim() {
        return new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60(1),
                        ShooterConstants.SHOOTER_MOI,
                        ShooterConstants.SHOOTER_GEAR_RATIO),
                DCMotor.getKrakenX60(1));
    }
}
