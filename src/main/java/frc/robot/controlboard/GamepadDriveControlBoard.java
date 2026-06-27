package frc.robot.controlboard;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.util.CommandSimXboxController;

public class GamepadDriveControlBoard implements IDriveControlBoard {
    private static GamepadDriveControlBoard instance = null;

    public static GamepadDriveControlBoard getInstance() {
        if (instance == null) {
            instance = new GamepadDriveControlBoard();
        }

        return instance;
    }

    private final CommandXboxController controller;

    private GamepadDriveControlBoard() {
        if (Robot.isSimulation()) {
            controller = new CommandSimXboxController(Constants.kDriveGamepadPort);
        } else {
            controller = new CommandXboxController(Constants.kDriveGamepadPort);
        }
    }

    @Override
    public double getThrottle() {
        return -(Math.pow(Math.abs(controller.getLeftY()), 1.5))
                * Math.signum(controller.getLeftY());
    }

    @Override
    public double getStrafe() {
        return -(Math.pow(Math.abs(controller.getLeftX()), 1.5))
                * Math.signum(controller.getLeftX());
    }

    @Override
    public double getRotation() {
        return -(Math.pow(Math.abs(controller.getRightX()), 2.0))
                * Math.signum(controller.getRightX());
    }

    @Override
    public double getRotationY() {
        return -(Math.pow(Math.abs(controller.getRightY()), 2.0))
                * Math.signum(controller.getRightY());
    }

    @Override
    public Trigger resetGyro() {
        return controller.back().and(controller.start().negate());
    }
}
