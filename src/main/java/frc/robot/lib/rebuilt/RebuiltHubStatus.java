package frc.robot.lib.rebuilt;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

/** Computes the alliance Hub state using the 2026 FMS game data and match clock. */
public final class RebuiltHubStatus {
    private RebuiltHubStatus() {}

    /**
     * Returns whether this robot's alliance Hub is active. Missing or invalid FMS game data is
     * treated as active, matching the WPILib 2026 game-data guidance for the early teleop window.
     */
    public static boolean isOwnHubActive() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }

        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) {
            return true;
        }

        boolean redInactiveFirst;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                return true;
            }
        }

        boolean shiftOneActive =
                switch (alliance.get()) {
                    case Red -> !redInactiveFirst;
                    case Blue -> redInactiveFirst;
                };

        double matchTimeSeconds = DriverStation.getMatchTime();
        if (matchTimeSeconds > 130.0) {
            return true;
        } else if (matchTimeSeconds > 105.0) {
            return shiftOneActive;
        } else if (matchTimeSeconds > 80.0) {
            return !shiftOneActive;
        } else if (matchTimeSeconds > 55.0) {
            return shiftOneActive;
        } else if (matchTimeSeconds > 30.0) {
            return !shiftOneActive;
        } else {
            return true;
        }
    }
}
