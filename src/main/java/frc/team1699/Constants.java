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
        public static final int kOperatorControllerPort = 1;
    }

    public static class ClimberConstants {
        // CAN IDs of winch motors
        public static final int kPortWinchID = -1;
        public static final int kStarWinchID = -1;
        public static final double kClimberSpeed = 0.5; // -1 to 1
    }

    public static class IndexerConstants {
        public static final int kMotorID = -1;
        public static final double kIndexerSpeed = 0.3;
    }

    public static class IntakeConstants {
        public static final int kDeckMotorID = 31;
        public static final int kBilgeMotorID = 32;
        public static final double kDeckSpeed = -0.75;
        public static final double kBilgeSpeed = -0.75;
    }

    public static class PivoterConstants {
        public static final int kMotorID = -1;
    }

    public static class ShooterConstants {
        public static final int kTopMotorID = -1;
        public static final int kBottomMotorID = -1;
    }

    public static class ManipulatorConstants {
        public static final double kSpeakerSubwooferSpeed = 0.0;
        public static final double kSpeakerStageSpeed = 0.0;
        public static final double kAmpSpeed = 0.0;
        public static final double kTrapSpeed = 0.0;
    }

    public static class SwerveConstants {
// photonvision-heading
   //     public static final double kDeadband = .15;
//Updated upstream
      //  public static final double kDeadband = .2;
      //  public static final double kMaxspeed = Units.feetToMeters(15,1);
      //  public static final double kMaxRetationalSpeed = Units.feetToMeters(10); // DOES THIS NEED TO BE DIFFRENT THAN MAX SPEED? 
        public static final double kDeadband = .02;
//main
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
// photonvision-heading
      //  public static final String kCameraName = "LifeCamOne";
//
        public static final String kCameraName = "Arducam_OV9281_USB_Camera";
// main
        public static final Transform3d kCameraPosition = new Transform3d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(14.25),
            Units.inchesToMeters(4), 
            new Rotation3d(0, Units.degreesToRadians(45), 0)
        );
        public static final AprilTagFields kAprilTagField = AprilTagFields.k2024Crescendo;
    }
}