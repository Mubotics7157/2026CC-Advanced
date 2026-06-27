package frc.robot.lib.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import frc.robot.lib.drivers.CANDeviceId;

public class CanCoderConfig {
    public CANDeviceId CANID;
    public CANcoderConfiguration config = new CANcoderConfiguration();
}
