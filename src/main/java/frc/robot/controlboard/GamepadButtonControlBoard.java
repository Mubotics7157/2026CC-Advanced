package frc.robot.controlboard;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.util.CommandSimXboxController;

public class GamepadButtonControlBoard implements IButtonControlBoard {
    private static GamepadButtonControlBoard instance = null;

    public static GamepadButtonControlBoard getInstance() {
        if (instance == null) {
            instance = new GamepadButtonControlBoard();
        }
        return instance;
    }

    private final CommandXboxController controller;

    @SuppressWarnings("unused")
    private GamepadButtonControlBoard() {
        if (Constants.kForceDriveGamepad
                || DriverStation.getJoystickIsXbox(Constants.kDriveGamepadPort)) {
            if (Robot.isSimulation()) {
                controller = new CommandSimXboxController(Constants.kDriveGamepadPort);
            } else {
                controller = new CommandXboxController(Constants.kDriveGamepadPort);
            }
        } else {
            controller = new CommandXboxController(Constants.kOperatorControllerPort);
        }
    }

    @Override
    public Trigger getWantToXWheels() {
        return controller.start().and(controller.back().negate());
    }

    @Override
    public Trigger getWantToAutoAlign() {
        return controller.start();
    }
}
