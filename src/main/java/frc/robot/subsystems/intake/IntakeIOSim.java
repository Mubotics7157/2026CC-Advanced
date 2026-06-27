package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
    private static final double SIM_LOOP_PERIOD = 0.02;
    private static final double MAX_VOLTAGE = 12.0;
    private static final double EXTENSION_KP = 8.0;
    private static final double EXTENSION_MOI = 0.01;
    private static final double ROLLER_MOI = 0.001;

    private final DCMotorSim rollerSim;
    private final DCMotorSim extensionSim;

    private double rollerAppliedVolts = 0.0;
    private double extensionAppliedVolts = 0.0;
    private double rollerVelocitySetpoint = 0.0;
    private double extensionPositionSetpoint = 0.0;

    public IntakeIOSim() {
        rollerSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                DCMotor.getKrakenX60(1),
                                ROLLER_MOI,
                                IntakeConstants.ROLLER_GEAR_RATIO),
                        DCMotor.getKrakenX60(1));
        extensionSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                DCMotor.getKrakenX60(1),
                                EXTENSION_MOI,
                                IntakeConstants.EXTENSION_MOTOR_TO_PINION_RATIO),
                        DCMotor.getKrakenX60(1));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        rollerSim.update(SIM_LOOP_PERIOD);
        extensionSim.update(SIM_LOOP_PERIOD);

        double extensionMeters = pinionRadiansToMeters(extensionSim.getAngularPositionRad());
        if (extensionMeters < IntakeConstants.EXTENSION_RETRACTED_POSITION_METERS) {
            extensionMeters = IntakeConstants.EXTENSION_RETRACTED_POSITION_METERS;
            extensionSim.setState(metersToPinionRadians(extensionMeters), 0.0);
        } else if (extensionMeters > IntakeConstants.EXTENSION_EXTENDED_POSITION_METERS) {
            extensionMeters = IntakeConstants.EXTENSION_EXTENDED_POSITION_METERS;
            extensionSim.setState(metersToPinionRadians(extensionMeters), 0.0);
        }

        inputs.rollerPosition = rollerSim.getAngularPositionRad();
        inputs.rollerVelocity = rollerSim.getAngularVelocityRadPerSec();
        inputs.rollerVoltage = rollerAppliedVolts;
        inputs.rollerCurrent = rollerSim.getCurrentDrawAmps();

        inputs.extensionPosition = extensionMeters;
        inputs.extensionVelocity =
                pinionRadiansToMeters(extensionSim.getAngularVelocityRadPerSec());
        inputs.extensionVoltage = extensionAppliedVolts;
        inputs.extensionCurrent = extensionSim.getCurrentDrawAmps();

        inputs.rollerVelocitySetpoint = rollerVelocitySetpoint;
        inputs.extensionPositionSetpoint = extensionPositionSetpoint;
    }

    @Override
    public void setRollerVelocity(double velocityRadPerSec) {
        rollerVelocitySetpoint = velocityRadPerSec;
        double error = velocityRadPerSec - rollerSim.getAngularVelocityRadPerSec();
        rollerAppliedVolts = MathUtil.clamp(error * 0.1, -MAX_VOLTAGE, MAX_VOLTAGE);
        rollerSim.setInputVoltage(rollerAppliedVolts);
    }

    @Override
    public void setExtensionVoltage(double voltage) {
        extensionAppliedVolts = MathUtil.clamp(voltage, -MAX_VOLTAGE, MAX_VOLTAGE);
        extensionSim.setInputVoltage(extensionAppliedVolts);
    }

    @Override
    public void setExtensionPosition(double positionMeters) {
        extensionPositionSetpoint = positionMeters;
        double error = positionMeters - pinionRadiansToMeters(extensionSim.getAngularPositionRad());
        setExtensionVoltage(error * EXTENSION_KP);
    }

    @Override
    public void stopRoller() {
        rollerVelocitySetpoint = 0.0;
        rollerAppliedVolts = 0.0;
        rollerSim.setInputVoltage(0.0);
    }

    @Override
    public void stopExtensionMotor() {
        setExtensionVoltage(0.0);
    }

    @Override
    public void setExtensionEncoderPosition(double positionMeters) {
        extensionSim.setState(
                metersToPinionRadians(positionMeters), extensionSim.getAngularVelocityRadPerSec());
    }

    private static double metersToPinionRadians(double meters) {
        return meters * IntakeConstants.EXTENSION_PINION_ROTATIONS_PER_METER * 2.0 * Math.PI;
    }

    private static double pinionRadiansToMeters(double radians) {
        return radians / (IntakeConstants.EXTENSION_PINION_ROTATIONS_PER_METER * 2.0 * Math.PI);
    }
}
