package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Desktop approximation of the roller and rotating forebar. */
public class IntakeIOSim implements IntakeIO {
    private static final double LOOP_PERIOD = 0.02;
    private final DCMotorSim rollerSim =
            new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            DCMotor.getKrakenX60(1), .001, IntakeConstants.ROLLER_GEAR_RATIO),
                    DCMotor.getKrakenX60(1));
    private final DCMotorSim armSim =
            new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(
                            DCMotor.getKrakenX60(1), .01, IntakeConstants.ARM_GEAR_RATIO),
                    DCMotor.getKrakenX60(1));
    private double rollerAppliedVolts, armAppliedVolts, rollerVelocitySetpoint, armPositionSetpoint;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        rollerSim.update(LOOP_PERIOD);
        armSim.update(LOOP_PERIOD);
        double armPosition = armSim.getAngularPositionRad();
        if (armPosition > IntakeConstants.ARM_STOWED_POSITION_RAD) {
            armPosition = IntakeConstants.ARM_STOWED_POSITION_RAD;
            armSim.setState(armPosition, 0.0);
        } else if (armPosition < IntakeConstants.ARM_DEPLOYED_POSITION_RAD) {
            armPosition = IntakeConstants.ARM_DEPLOYED_POSITION_RAD;
            armSim.setState(armPosition, 0.0);
        }
        inputs.rollerPosition = rollerSim.getAngularPositionRad();
        inputs.rollerVelocity = rollerSim.getAngularVelocityRadPerSec();
        inputs.rollerVoltage = rollerAppliedVolts;
        inputs.rollerCurrent = rollerSim.getCurrentDrawAmps();
        inputs.armPosition = armPosition;
        inputs.armVelocity = armSim.getAngularVelocityRadPerSec();
        inputs.armVoltage = armAppliedVolts;
        inputs.armCurrent = armSim.getCurrentDrawAmps();
        inputs.rollerVelocitySetpoint = rollerVelocitySetpoint;
        inputs.armPositionSetpoint = armPositionSetpoint;
    }

    @Override
    public void setRollerVelocity(double velocityRadPerSec) {
        rollerVelocitySetpoint = velocityRadPerSec;
        rollerAppliedVolts =
                MathUtil.clamp(
                        (velocityRadPerSec - rollerSim.getAngularVelocityRadPerSec()) * .1,
                        -12,
                        12);
        rollerSim.setInputVoltage(rollerAppliedVolts);
    }

    @Override
    public void setArmVoltage(double voltage) {
        armAppliedVolts = MathUtil.clamp(voltage, -12, 12);
        armSim.setInputVoltage(armAppliedVolts);
    }

    @Override
    public void setArmPosition(double positionRad) {
        armPositionSetpoint =
                MathUtil.clamp(
                        positionRad,
                        IntakeConstants.ARM_DEPLOYED_POSITION_RAD,
                        IntakeConstants.ARM_STOWED_POSITION_RAD);
        setArmVoltage((armPositionSetpoint - armSim.getAngularPositionRad()) * 8);
    }

    @Override
    public void stopRoller() {
        rollerVelocitySetpoint = 0;
        rollerAppliedVolts = 0;
        rollerSim.setInputVoltage(0);
    }

    @Override
    public void stopArmMotor() {
        setArmVoltage(0);
    }

    @Override
    public void setArmEncoderPosition(double positionRad) {
        armSim.setState(positionRad, armSim.getAngularVelocityRadPerSec());
    }
}
