# Drive System Identification

The `Auto/Mode` chooser exposes quasistatic and dynamic tests in both directions for drive
translation, steering, and robot rotation. Run each of the four tests once for a mechanism. Phoenix
SignalLogger keeps one `.hoot` log open across those tests, then closes it after the full set. An
incomplete set is closed after two minutes without another test or when switching to a different
mechanism. Tests shorter than three seconds reset the active log because they do not provide enough
data for a useful fit.

## Test behavior

- **Drive translation** applies voltage to all drive motors with the modules pointed forward. The
  routine waits for measured alignment and low wheel/steer speed; it skips the test if the modules
  are not ready within three seconds. The WPILib log records each module separately.
- **Steering** applies voltage to all azimuth motors and logs each module's continuous azimuth,
  velocity, and applied voltage. Make sure the wheels can turn freely and the robot is secured before
  running this test.
- **Robot rotation** uses Phoenix's `SysIdSwerveRotation` request, which ramps angular rate rather
  than motor voltage. In SysId's log, requested angular rate (rad/s) is stored in the input/voltage
  field, with continuous Pigeon yaw and angular velocity logged in radians and radians/second.

All tests have a ten-second limit. Disable the robot to interrupt a test. Convert the resulting
`.hoot` file to WPILOG with Tuner X or `owlet`, then select the matching mechanism and motor entries
in WPILib SysId. Translation and rotation tests move the robot, so run them in a clear area with
room to travel in both directions.
