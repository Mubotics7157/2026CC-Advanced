package frc.robot;

import edu.wpi.first.math.util.Units;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

public class Constants {
    /** Set by `Main` when running log replay. */
    public static boolean kIsReplay = false;

    public enum SimControllerType {
        XBOX,
        DUAL_SENSE
    }

    public static final SimControllerType kSimControllerType = SimControllerType.XBOX;

    // Controls
    public static final boolean kForceDriveGamepad = true;
    public static final int kDriveGamepadPort = 0;
    public static final int kOperatorControllerPort = 2;
    public static final double kSteerJoystickDeadband = 0.05;

    // Simulation
    public static final boolean useMapleSim = true;

    // Robot physical constants //TODO: Update these values for the bot
    public static final double kRobotWidth = Units.inchesToMeters(26.5);
    public static final double kRobotMassKg = 39.0;
    public static final double kRobotMomentOfInertia = 1.3417722158; // kg * m^2 did this one
    public static final double kCOGHeightMeters = 0.2;

    // Bot identity
    public static final String kPracticeBotMacAddress = "00:80:2F:33:BF:BB";
    public static final boolean kIsPracticeBot = hasMacAddress(kPracticeBotMacAddress);

    public static final class AutoConstants {
        public static final double kPXYController = 5.0;
        public static final double kPLTEController = 3.0;
        public static final double kPCTEController = 6.0;
        public static final double kPThetaController = 5.0;

        public static final double kMaxEndPathVelocity = 2.0; // m/s
    }

    public static boolean hasMacAddress(final String macAddress) {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis == null) continue;
                byte[] mac = nis.getHardwareAddress();
                if (mac == null) continue;

                StringBuilder deviceMac = new StringBuilder();
                for (int i = 0; i < mac.length; i++) {
                    deviceMac.append(
                            String.format("%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
                }
                if (macAddress.equals(deviceMac.toString())) {
                    return true;
                }
            }
        } catch (SocketException ignored) {
            // Ignore and assume not the practice bot.
        }
        return false;
    }
}
