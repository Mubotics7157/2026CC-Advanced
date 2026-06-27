package frc.robot.controlboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControlBoard implements IDriveControlBoard, IButtonControlBoard {
    private static ControlBoard instance = null;

    public static ControlBoard getInstance() {
        if (instance == null) {
            instance = new ControlBoard();
        }
        return instance;
    }

    private final IDriveControlBoard driveControlBoard;
    private final IButtonControlBoard buttonControlBoard;

    private ControlBoard() {
        driveControlBoard = GamepadDriveControlBoard.getInstance();
        buttonControlBoard = GamepadButtonControlBoard.getInstance();
    }

    @Override
    public double getThrottle() {
        return driveControlBoard.getThrottle();
    }

    @Override
    public double getStrafe() {
        return driveControlBoard.getStrafe();
    }

    @Override
    public double getRotation() {
        return driveControlBoard.getRotation();
    }

    @Override
    public double getRotationY() {
        return driveControlBoard.getRotationY();
    }

    @Override
    public Trigger resetGyro() {
        return driveControlBoard.resetGyro();
    }

    @Override
    public Trigger getWantToXWheels() {
        return buttonControlBoard.getWantToXWheels();
    }

    @Override
    public Trigger getWantToAutoAlign() {
        return buttonControlBoard.getWantToAutoAlign();
    }
}
