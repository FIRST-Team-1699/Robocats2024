package frc.team1699;
public class Constants {
    // The DI/O port where the LED strip is plugged in
    public static final int kLEDPort = 0;

    public static class InputConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class SwerveConstants {
        public static final double kDeadband = .15;
        public static final double kMaxSpeed = Units.feetToMeters(15.1);
        public static final double kMaxRotationalSpeed = 4;
        public static final double kTrackWidth = Units.inchesToMeters(20.5);
        public static final double kHalfTrackWidth = kTrackWidth / 2.0;
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(kHalfTrackWidth, kHalfTrackWidth),
            new Translation2d(-kHalfTrackWidth, kHalfTrackWidth),
            new Translation2d(kHalfTrackWidth, -kHalfTrackWidth,
            new Translation2d(-kHalfTrackWidth, -kHalfTrackWidth)
        );
    }
}