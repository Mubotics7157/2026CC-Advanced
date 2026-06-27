package frc.robot.controlboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IButtonControlBoard {
    Trigger getWantToXWheels();

    Trigger getWantToAutoAlign();
}
