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
        public static final double kP = 0.5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;
        public static final double kPortClimberTopPosition = 440;
        public static final double kPortClimberBottomPosition = 0.0;
        public static final double kStarClimberTopPosition = 440;
        public static final double kStarClimberBottomPosition = 0.0;
    }

    public static class IndexerConstants {
        public static final int kMotorID = 16;
        public static final double kIndexerSpeed = 0.25;
        public static final double kFeedSpeed = 0.75;
        public static final int kBeamBreakID = 0;
    }

    public static class IntakeConstants {
        public static final int kDeckMotorID = 31;
        public static final int kBilgeMotorID = 32;
        public static final double kDeckSpeed = -0.75;
        public static final double kBilgeSpeed = -0.75;
        // for later
        // indexer = 1 x 1000/20
        // bottom intake = .5 x 1000/7
        // top intake = 1 x 1000/28
    }

    public static class PivoterConstants {
        public static final int kMotorID = 35;
        public static final double kEncoderOffset = 0.0;
        public static final double kTolerance = 2.0;
        public static final double kP = 0.03;
        public static final double kI = 0.0;
        public static final double kD = 0.02;
        public static final double kMinAngle = 27.0;
        public static final double kMaxAngle = 130.0; 
    }

    public static class ShooterConstants {
        public static final int kTopMotorID = 33;
        public static final int kBottomMotorID = 34;
    }

    public static class ManipulatorConstants {
        public static final double kSpeakerSubwooferSpeed = 50;
        public static final double kSpeakerSubwooferAngle = 50;
        public static final double kSpeakerLLSpeed = 70;
        public static final double kSpeakerStageAngle = 35;
        public static final double kAmpTopSpeed = 20;
        public static final double kAmpBottomSpeed = 25;
        public static final double kAmpAngle = 110;
        public static final double kTrapSpeed = 0.0;
        public static final double kTrapAngle = 80;
        public static final double kIdleAngle = 50;
        public static final double kIntakeAngle = 42;
    }

    public static class SwerveConstants {
        public static final double kDeadband = .02;
        public static final double kSlowStrafeCoefficient = 1;
        public static final double kADSStrafeCoefficient = .6;
        public static final double kMaxSpeed = Units.feetToMeters(15.1) * kSlowStrafeCoefficient;
        public static final double kSlowRotationCoefficient = 1;
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
        public static final PhysicalConfig kCameraPosition = new PhysicalConfig().withTranslation(Units.inchesToMeters(-10), 0.0, Units.inchesToMeters(12)).withRotation(55.0, 0.0, 180.0);
        public static final double kSpeakerAimHeight = Units.inchesToMeters(70);
    }
}