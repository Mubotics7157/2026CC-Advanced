package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
    private static final double SIM_LOOP_PERIOD = 0.02;
    private static final double MAX_VOLTAGE = 12.0;

    private final DCMotorSim leftSim;
    private final DCMotorSim middleSim;
    private final DCMotorSim rightSim;

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
        leftAppliedVolts = calculateVoltage(leftSim, velocityRadPerSec);
        middleAppliedVolts = calculateVoltage(middleSim, velocityRadPerSec);
        rightAppliedVolts = calculateVoltage(rightSim, velocityRadPerSec);

        leftSim.setInputVoltage(leftAppliedVolts);
        middleSim.setInputVoltage(middleAppliedVolts);
        rightSim.setInputVoltage(rightAppliedVolts);
    }

    @Override
    public void runVelocityDutyCycleBangBang(double velocityRadPerSec) {
        velocitySetpoint = velocityRadPerSec;
        leftAppliedVolts =
                bangBangVoltage(leftSim.getAngularVelocityRadPerSec(), velocityRadPerSec);
        middleAppliedVolts =
                bangBangVoltage(middleSim.getAngularVelocityRadPerSec(), velocityRadPerSec);
        rightAppliedVolts =
                bangBangVoltage(rightSim.getAngularVelocityRadPerSec(), velocityRadPerSec);
        leftSim.setInputVoltage(leftAppliedVolts);
        middleSim.setInputVoltage(middleAppliedVolts);
        rightSim.setInputVoltage(rightAppliedVolts);
    }

    @Override
    public void runTorqueCurrent(double torqueCurrentAmps) {
        double voltage =
                MathUtil.clamp(
                        torqueCurrentAmps
                                / ShooterConstants.SHOOTER_PEAK_TORQUE_CURRENT_AMPS.get()
                                * MAX_VOLTAGE,
                        0.0,
                        MAX_VOLTAGE);
        leftAppliedVolts = voltage;
        middleAppliedVolts = voltage;
        rightAppliedVolts = voltage;
        leftSim.setInputVoltage(voltage);
        middleSim.setInputVoltage(voltage);
        rightSim.setInputVoltage(voltage);
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
        hoodServoPosition = ShooterConstants.HOOD_SERVO_MIN_POSITION;
        leftAppliedVolts = 0.0;
        middleAppliedVolts = 0.0;
        rightAppliedVolts = 0.0;
        leftSim.setInputVoltage(0.0);
        middleSim.setInputVoltage(0.0);
        rightSim.setInputVoltage(0.0);
    }

    private static DCMotorSim createSim() {
        return new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60(1),
                        ShooterConstants.SHOOTER_MOI,
                        ShooterConstants.SHOOTER_GEAR_RATIO),
                DCMotor.getKrakenX60(1));
    }

    private static double calculateVoltage(DCMotorSim sim, double velocityRadPerSec) {
        double error = velocityRadPerSec - sim.getAngularVelocityRadPerSec();
        return MathUtil.clamp(error * 0.1, -MAX_VOLTAGE, MAX_VOLTAGE);
    }

    private static double bangBangVoltage(double measuredVelocity, double setpoint) {
        return measuredVelocity < setpoint ? MAX_VOLTAGE : 0.0;
    }
}
