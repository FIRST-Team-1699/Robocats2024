package frc.team1699;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.team1699.lib.vision.Limelight.PhysicalConfig;

public class Constants {
    // The DI/O port where the LED strip is plugged in
    public static final int kLEDPort = 0;

    public static class InputConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static class ClimberConstants {
        // CAN IDs of winch motors
        public static final int kPortWinchID = 12;
        public static final int kStarWinchID = 23;
        public static final double kClimberSpeed = 0.5; // -1 to 1
    }

    public static class IndexerConstants {
        public static final int kMotorID = -1;
        public static final double kIndexerSpeed = 0.3;
        public static final int kBeamBreakID = 0;
    }

    public static class IntakeConstants {
        public static final int kDeckMotorID = 31;
        public static final int kBilgeMotorID = 32;
        public static final double kDeckSpeed = -0.75;
        public static final double kBilgeSpeed = -0.75;
    }

    public static class PivoterConstants {
        public static final int kMotorID = -1;
        public static final double kEncoderOffset = 0.0;
        public static final double kTolerance = 2.0;
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kMinAngle = 45.0;
        public static final double kMaxAngle = 100.0; 
    }

    public static class ShooterConstants {
        public static final int kTopMotorID = -1;
        public static final int kBottomMotorID = -1;
    }

    public static class ManipulatorConstants {
        public static final double kSpeakerSubwooferSpeed = 0.0;
        public static final double kSpeakerSubwooferAngle = 0.0;
        public static final double kSpeakerStageSpeed = 0.0;
        public static final double kSpeakerStageAngle = 0.0;
        public static final double kAmpSpeed = 0.0;
        public static final double kAmpAngle = 0.0;
        public static final double kTrapSpeed = 0.0;
        public static final double kTrapAngle = 0.0;
        public static final double kIdleAngle = 0.0;
        public static final double kIntakeAngle = 0.0;
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
        public static final String kLimelightName = "limelight";
        public static final int kPipelineID = 0;
        // FORWARD VALUE IS TEMPORARY
        public static final PhysicalConfig kCameraPosition = new PhysicalConfig().withTranslation(-10, 0.0, 8.5).withRotation(55.0, 0.0, 180.0);
        public static final double kSpeakerAimHeight = Units.inchesToMeters(81);
    }
}