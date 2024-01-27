package frc.team1699;
public class Constants {
    // The DI/O port where the LED strip is plugged in
    public static final int kLEDPort = 0;
    public static class SwerveConstants {
<<<<<<< Updated upstream
        public static final double kDeadband = .2;
        public static final double kMaxspeed = Units.feetToMeters(15,1);
        public static final double kMaxRetationalSpeed = Units.feetToMeters(10); // DOES THIS NEED TO BE DIFFRENT THAN MAX SPEED? 
=======
        public static final double kDeadband = .02;
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
        public static final String kCameraName = "Arducam_OV9281_USB_Camera";
        public static final Transform3d kCameraPosition = new Transform3d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(14.25),
            Units.inchesToMeters(4), 
            new Rotation3d(0, Units.degreesToRadians(45), 0)
        );
        public static final AprilTagFields kAprilTagField = AprilTagFields.k2024Crescendo;
>>>>>>> Stashed changes
    }
}