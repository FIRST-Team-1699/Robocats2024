package frc.team1699;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    // The DI/O port where the LED strip is plugged in
    public static final int kLEDPort = 0;

    public static class InputConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class SwerveConstants {
        public static final double kDeadband = .15;
        public static final double kSlowStrafeCoefficient = .75;
        public static final double kMaxSpeed = Units.feetToMeters(15.1) * kSlowStrafeCoefficient;
        public static final double kSlowRotationCoefficient = .60;
        public static final double kMaxRotationalSpeed = Units.degreesToRadians(720) * kSlowRotationCoefficient;
        public static final double kTrackWidth = Units.inchesToMeters(20.5);
        public static final double kHalfTrackWidth = kTrackWidth / 2.0;
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(kHalfTrackWidth, kHalfTrackWidth),
            new Translation2d(-kHalfTrackWidth, kHalfTrackWidth),
            new Translation2d(kHalfTrackWidth, -kHalfTrackWidth),
            new Translation2d(-kHalfTrackWidth, -kHalfTrackWidth)
        );
    }

    public static class VisionConstants {
        public static final String kCameraName = "LifeCamOne";
        public static final Transform3d kCameraPosition = new Transform3d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(14.25),
            Units.inchesToMeters(4), 
            new Rotation3d(0, Units.degreesToRadians(45), 0)
        );
        public static final AprilTagFields kAprilTagField = AprilTagFields.k2024Crescendo;
    }
}