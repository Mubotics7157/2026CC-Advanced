// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.lib.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(690.876);
    public static final double fieldWidth = Units.inchesToMeters(317);
    public static final double startingLineX =
            Units.inchesToMeters(299.438); // Measured from the inside of starting line

    public static class Processor {
        public static final Pose2d centerFace =
                new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
    }

    public static class Barge {
        public static final Translation2d farCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
        public static final Translation2d middleCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
        public static final Translation2d closeCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

        // Measured from floor to bottom of cage
        public static final double deepHeight = Units.inchesToMeters(3.125);
        public static final double shallowHeight = Units.inchesToMeters(30.125);
    }

    public static class CoralStation {
        public static final Pose2d leftCenterFace =
                new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units.inchesToMeters(291.176),
                        Rotation2d.fromDegrees(90 - 144.011));
        public static final Pose2d rightCenterFace =
                new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units.inchesToMeters(25.824),
                        Rotation2d.fromDegrees(144.011 - 90));
    }

    @SuppressWarnings("unchecked")
    public static class Reef {
        public static final Translation2d center =
                new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
        public static final double faceToZoneLine =
                Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

        public static final Pose2d[] centerFaces =
                new Pose2d[6]; // Starting facing the driver station in clockwise order
        public static final List<Map<ReefHeight, Pose3d>> branchPositions =
                new ArrayList<>(); // Starting at the right branch facing the driver station in
        // clockwise
        public static final List<Map<ReefHeight, Pose3d>> branchTipPositions =
                new ArrayList<>(); // Starting at the right branch facing the driver station in

        // clockwise

        static {
            // Initialize faces
            centerFaces[0] =
                    new Pose2d(
                            Units.inchesToMeters(144.003),
                            Units.inchesToMeters(158.500),
                            Rotation2d.fromDegrees(180));
            centerFaces[1] =
                    new Pose2d(
                            Units.inchesToMeters(160.373),
                            Units.inchesToMeters(186.857),
                            Rotation2d.fromDegrees(120));
            centerFaces[2] =
                    new Pose2d(
                            Units.inchesToMeters(193.116),
                            Units.inchesToMeters(186.858),
                            Rotation2d.fromDegrees(60));
            centerFaces[3] =
                    new Pose2d(
                            Units.inchesToMeters(209.489),
                            Units.inchesToMeters(158.502),
                            Rotation2d.fromDegrees(0));
            centerFaces[4] =
                    new Pose2d(
                            Units.inchesToMeters(193.118),
                            Units.inchesToMeters(130.145),
                            Rotation2d.fromDegrees(-60));
            centerFaces[5] =
                    new Pose2d(
                            Units.inchesToMeters(160.375),
                            Units.inchesToMeters(130.144),
                            Rotation2d.fromDegrees(-120));

            // Initialize branch positions
            for (int face = 0; face < 6; face++) {
                Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
                Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
                for (var level : ReefHeight.values()) {
                    Pose2d poseDirection =
                            new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
                    double adjustX = Units.inchesToMeters(30.738);
                    double adjustY = Units.inchesToMeters(6.469);

                    fillRight.put(
                            level,
                            new Pose3d(
                                    new Translation3d(
                                            poseDirection
                                                    .transformBy(
                                                            new Transform2d(
                                                                    adjustX,
                                                                    adjustY,
                                                                    new Rotation2d()))
                                                    .getX(),
                                            poseDirection
                                                    .transformBy(
                                                            new Transform2d(
                                                                    adjustX,
                                                                    adjustY,
                                                                    new Rotation2d()))
                                                    .getY(),
                                            level.height),
                                    new Rotation3d(
                                            0,
                                            Units.degreesToRadians(level.pitch),
                                            poseDirection.getRotation().getRadians())));
                    fillLeft.put(
                            level,
                            new Pose3d(
                                    new Translation3d(
                                            poseDirection
                                                    .transformBy(
                                                            new Transform2d(
                                                                    adjustX,
                                                                    -adjustY,
                                                                    new Rotation2d()))
                                                    .getX(),
                                            poseDirection
                                                    .transformBy(
                                                            new Transform2d(
                                                                    adjustX,
                                                                    -adjustY,
                                                                    new Rotation2d()))
                                                    .getY(),
                                            level.height),
                                    new Rotation3d(
                                            0,
                                            Units.degreesToRadians(level.pitch),
                                            poseDirection.getRotation().getRadians())));
                }
                branchPositions.add(fillRight);
                branchPositions.add(fillLeft);
            }

            // Mirror for red alliance
            for (Map<ReefHeight, Pose3d> blueBranch : branchPositions.toArray(new Map[0])) {
                Map<ReefHeight, Pose3d> redBranch = new HashMap<>();
                for (Map.Entry<ReefHeight, Pose3d> entry : blueBranch.entrySet()) {
                    Pose3d bluePose = entry.getValue();
                    redBranch.put(
                            entry.getKey(),
                            new Pose3d(
                                    Util.flipRedBlue(
                                            new Translation3d(
                                                    bluePose.getX(),
                                                    bluePose.getY(),
                                                    bluePose.getZ())),
                                    new Rotation3d(
                                            0,
                                            -bluePose.getRotation().getY(),
                                            -bluePose.getRotation().getZ())));
                }
                branchPositions.add(redBranch);
            }
            for (int face = 0; face < 6; face++) {
                Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
                Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
                for (var level : ReefHeight.values()) {
                    Pose2d poseDirection =
                            new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
                    double adjustX = Units.inchesToMeters(-2);
                    double adjustY = Units.inchesToMeters(6.469);

                    fillRight.put(
                            level,
                            new Pose3d(
                                    new Translation3d(
                                            poseDirection
                                                    .transformBy(
                                                            new Transform2d(
                                                                    adjustX,
                                                                    adjustY,
                                                                    new Rotation2d()))
                                                    .getX(),
                                            poseDirection
                                                    .transformBy(
                                                            new Transform2d(
                                                                    adjustX,
                                                                    adjustY,
                                                                    new Rotation2d()))
                                                    .getY(),
                                            level.height),
                                    new Rotation3d(
                                            0,
                                            Units.degreesToRadians(level.pitch),
                                            poseDirection.getRotation().getRadians())));
                    fillLeft.put(
                            level,
                            new Pose3d(
                                    new Translation3d(
                                            poseDirection
                                                    .transformBy(
                                                            new Transform2d(
                                                                    adjustX,
                                                                    -adjustY,
                                                                    new Rotation2d()))
                                                    .getX(),
                                            poseDirection
                                                    .transformBy(
                                                            new Transform2d(
                                                                    adjustX,
                                                                    -adjustY,
                                                                    new Rotation2d()))
                                                    .getY(),
                                            level.height),
                                    new Rotation3d(
                                            0,
                                            Units.degreesToRadians(level.pitch),
                                            poseDirection.getRotation().getRadians())));
                }
                branchTipPositions.add(fillRight);
                branchTipPositions.add(fillLeft);
            }
        }
    }

    public static class StagingPositions {
        // Measured from the center of the ice cream
        public static final Pose2d leftIceCream =
                new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
        public static final Pose2d middleIceCream =
                new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(170.5), new Rotation2d());
        public static final Pose2d rightIceCream =
                new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
    }

    public static class HPIntake {
        // HP Intake positions for blue alliance
        public static final Pose2d kStationA =
                new Pose2d(
                        new Translation2d(0.7571420669555664, 0.6461422443389893),
                        new Rotation2d(Math.PI / 4));

        public static final Pose2d kStationB =
                new Pose2d(
                        new Translation2d(0.7571420669555664, 7.364640235900879),
                        new Rotation2d(-Math.PI / 4));
    }

    public enum ReefHeight {
        L4(Units.inchesToMeters(72), -90),
        L3(Units.inchesToMeters(47.625), -35),
        L2(Units.inchesToMeters(31.875), -35),
        L1(Units.inchesToMeters(18), 0);

        ReefHeight(double height, double pitch) {
            this.height = height;
            this.pitch = pitch; // in degrees
        }

        public final double height;
        public final double pitch;
    }

    public enum BranchCode {
        A(0),
        B(1),
        C(2),
        D(3),
        E(4),
        F(5),
        G(6),
        H(7),
        L(8);

        BranchCode(int indexOffset) {
            this.indexOffset = indexOffset;
        }

        public final int indexOffset;
    }

    // AprilTag layout
    private static final AprilTagFieldLayout kAprilTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private static final int[] kAllowedTagIDs = {17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11};
    public static final AprilTagFieldLayout kAprilTagLayoutReefsOnly =
            new AprilTagFieldLayout(
                    kAprilTagLayout.getTags().stream()
                            .filter(
                                    tag ->
                                            Arrays.stream(kAllowedTagIDs)
                                                    .anyMatch(element -> element == tag.ID))
                            .toList(),
                    kAprilTagLayout.getFieldLength(),
                    kAprilTagLayout.getFieldWidth());

    public static final double kFieldWidthMeters = kAprilTagLayout.getFieldWidth();
    public static final double kFieldLengthMeters = kAprilTagLayout.getFieldLength();

    // Auto-align / scoring location geometry
    public static final Pose2d kFeederRightPose =
            new Pose2d(1.7196709632873535, 0.6049244999885559, Rotation2d.fromDegrees(60));
    public static final Pose2d kFeederLeftPose =
            new Pose2d(1.7196709632873535, 7.364640235900879, Rotation2d.fromDegrees(-60));
    public static final Pose2d kBargePose = new Pose2d(8.0, 6.0, Rotation2d.k180deg);

    public static final double kAutoAlignReefBuffer = 1.0; // m
    public static final double kAutoAlignReefBufferAuto = 1.5; // m
    public static final double kAutoAlignReefUnbuffer = 1.0; // m
    public static final double kAutoAlignFeederBufferAuto = 0.2; // m
    public static final double kAutoAlignFeederGroundBufferAuto = 0.2; // m
    public static final double kAutoAlignFeederGroundBufferFirstPathAuto = 0.5; // m
    public static final double kAutoAlignFeederBufferTeleop = 0.1; // m
    public static final double kAutoAlignReefBackoffDistance = -0.3; // m
    public static final double kAutoAlignAlgaeReefBackoffDistance = -0.65; // m

    public static final double kMidlineBuffer = 1.0;

    public static final Pose2d[] kBranchPoses = getBranchPoses();
    public static final Pose2d[] kAlgaePoses = getReefCenterPoses();

    public static Pose2d[] getReefCenterPoses() {
        Pose2d[] reefPoses = new Pose2d[Reef.centerFaces.length];
        for (int i = 0; i < reefPoses.length; i++) {
            Pose2d reefPose = Reef.centerFaces[i];
            Transform2d offsetTransform =
                    new Transform2d(
                            new Translation2d(kRobotWidth / 2.0 + kAutoAlignReefBuffer, 0.0),
                            Rotation2d.k180deg);
            reefPoses[i] = reefPose.plus(offsetTransform);
        }
        return reefPoses;
    }

    public static Pose2d[] getBranchPoses() {
        Pose2d[] branchPoses = new Pose2d[Reef.branchPositions.size()];
        for (int i = 0; i < branchPoses.length; i++) {
            Pose3d branchPose3d = Reef.branchPositions.get(i).get(ReefHeight.L3);
            Translation2d branchTranslation2d =
                    new Translation2d(branchPose3d.getX(), branchPose3d.getY());
            Rotation2d branchRotation2d = new Rotation2d(branchPose3d.getRotation().getZ());
            branchPoses[i] = new Pose2d(branchTranslation2d, branchRotation2d);

            Transform2d offsetTransform =
                    new Transform2d(
                            new Translation2d(kRobotWidth / 2.0 + kAutoAlignReefBuffer, 0.0),
                            Rotation2d.k180deg);
            branchPoses[i] = branchPoses[i].plus(offsetTransform);
        }
        return branchPoses;
    }

    // Robot width needed for pose calculations (passed from Constants)
    public static double kRobotWidth = Units.inchesToMeters(32.0);
}
