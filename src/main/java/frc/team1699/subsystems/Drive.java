package frc.team1699.subsystems;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.team1699.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
// i hate kevin he is so mean = true;
public class Drive {
    private DriveState currentState = DriveState.TELEOP_DRIVE;
    private DriveState wantedState = DriveState.TELEOP_DRIVE;
    private Timer trajTimer = new Timer();
    private PathPlannerTrajectory trajectory;
    private PPHolonomicDriveController driveController;
    private PIDConstants translationConstants = new PIDConstants(0.02);
    private PIDConstants rotationConstants = new PIDConstants(0.2);
    private boolean doneWithTraj = true;
    private PIDController headingLockController = new PIDController(.013, 0, 0);

    private SwerveDrive swerve;
    private XboxController controller;
    private Vision vision;
    private Field2d field;

    public Drive(XboxController controller) {
        try {
            this.swerve = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(Units.feetToMeters(15.1));
        } catch (IOException e) {
            System.out.print("Swerve build failed");
            e.printStackTrace(System.out);
        }
        this.controller = controller;
        this.driveController = new PPHolonomicDriveController(translationConstants, rotationConstants, SwerveConstants.kMaxSpeed, Units.inchesToMeters(24.75 / Math.sqrt(2)));
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
        headingLockController.enableContinuousInput(-180, 180);
        headingLockController.setTolerance(3);
        this.vision = Vision.getInstance();
        field = new Field2d();
    }

    private void teleopDrive() {
        // get controller inputs
        double vX = -controller.getLeftX();
        double vY = -controller.getLeftY();
        double vR = -controller.getRightX();

        // apply deadbands
        if(Math.abs(vX) < SwerveConstants.kDeadband) {
            vX = 0;
        }
        if(Math.abs(vY) < SwerveConstants.kDeadband) {
            vY = 0;
        }
        if(Math.abs(vR) < SwerveConstants.kDeadband) {
            vR = 0;
        }
        // scale outputs
        if(DriverStation.isTeleop()) {
            vX *= SwerveConstants.kMaxSpeed * SwerveConstants.kSlowStrafeCoefficient; 
            vY *= SwerveConstants.kMaxSpeed * SwerveConstants.kSlowStrafeCoefficient;
            vR *= SwerveConstants.kMaxRotationalSpeed * SwerveConstants.kSlowRotationCoefficient;
        } else {
            vX *= SwerveConstants.kMaxSpeed;
            vY *= SwerveConstants.kMaxSpeed;
            vR *= SwerveConstants.kMaxRotationalSpeed;
        }

        // drive swerve
        swerve.drive(new Translation2d(vY, vX), vR, true, false);
    }

    private void teleopDriveHeadingPID(double targetOffset) {
        // get controller inputs
        double vX = -controller.getLeftX();
        double vY = -controller.getLeftY();
        double vR = headingLockController.calculate(targetOffset, 0.0);

        // apply deadbands
        if(Math.abs(vX) < SwerveConstants.kDeadband) {
            vX = 0;
        }
        if(Math.abs(vY) < SwerveConstants.kDeadband) {
            vY = 0;
        }

        // scale outputs
        if(currentState == DriveState.TELEOP_SPEAKER_TRACK) {
            vX *= SwerveConstants.kMaxSpeed * SwerveConstants.kADSStrafeCoefficient;
            vY *= SwerveConstants.kMaxSpeed * SwerveConstants.kADSStrafeCoefficient;
            vR *= SwerveConstants.kMaxRotationalSpeed;
        } else {
            vX *= SwerveConstants.kMaxSpeed * SwerveConstants.kSlowStrafeCoefficient; 
            vY *= SwerveConstants.kMaxSpeed * SwerveConstants.kSlowStrafeCoefficient;
            vR *= SwerveConstants.kMaxRotationalSpeed;
        }
        
        // drive swerve
        swerve.drive(new Translation2d(vY, vX), vR, true, false);
    }

    public void setTrajectory(PathPlannerTrajectory trajectory) {
        this.trajectory = trajectory;
    }

    public void zeroGyro() {
        swerve.zeroGyro();
    } 

    // /** Manually set the module states
    //  * @param moduleStates
    //  * FL, FR, BL, BR
    //  */
    // private void setModuleStates(SwerveModuleState[] moduleStates) {
    //     swerve.setModuleStates(moduleStates, false);
    // }

    private void lock() {
        swerve.lockPose();
    }

    public boolean headingAimed() {
        if(vision.hasTargetInView() && getState() == DriveState.TELEOP_SPEAKER_TRACK && headingLockController.atSetpoint()) {
            return true;
        }
        return false;
    }

    private void driveTraj() {
        if(trajTimer.get() < trajectory.getTotalTimeSeconds()) {
            PathPlannerTrajectory.State targetState = trajectory.sample(trajTimer.get());
            ChassisSpeeds targetSpeeds = driveController.calculateRobotRelativeSpeeds(swerve.getPose(), targetState);
            swerve.drive(targetSpeeds);

            System.out.println("TARG SPED" + targetSpeeds);
            System.out.println("GYRO" + swerve.getOdometryHeading());
        } else {
            trajTimer.stop();
            doneWithTraj = true;
            setWantedState(DriveState.LOCK);
        }
    }

    // private void updateVisionData() {
    //     if(vision.hasTargetInView()) {
    //         swerve.addVisionMeasurement(vision.getPose2d(), Timer.getFPGATimestamp());
    //     }
    // } 

    public void update() {
        field.setRobotPose(getPose());
        // updateVisionData();
        switch (currentState) {
            case FOLLOW_TRAJ:
                driveTraj();
                break;
            case LOCK:
                lock();
                break;
            case TELEOP_DRIVE:
                teleopDrive();
                break;
            case TELEOP_SPEAKER_TRACK:
                teleopDriveHeadingPID(vision.getSpeakerX());
                break;
            case TELEOP_AMP_TRACK:
                teleopDriveHeadingPID(Rotation2d.fromDegrees(90).minus(getHeading()).getDegrees());
            break;
            default:
                break;
        }
    }

    private void handleStateTransition() {
        switch (wantedState) {
            case FOLLOW_TRAJ:
                trajTimer.reset();
                trajTimer.start();
                doneWithTraj = false;
                swerve.resetOdometry(trajectory.getInitialDifferentialPose());
                break;
            case LOCK:
                break;
            case TELEOP_DRIVE:
                break;
            case TELEOP_SPEAKER_TRACK:
                break;
            default:
                break;
        }
        currentState = wantedState;
    }

    public void setWantedState(DriveState state) {
        if(this.wantedState != state) {
            wantedState = state;
            handleStateTransition();
        }
    }

    public DriveState getState() {
        return this.currentState;
    }

    /** For auto
     * @param states
     * FL, FR, BL, BR
     */
    public void setModuleStates(SwerveModuleState[] states) {
        swerve.setModuleStates(states, false);
    }

    public Pose2d getPose() {
        return swerve.getPose();
    }

    public Rotation2d getHeading() {
        return swerve.getOdometryHeading();
    }

    public boolean doneWithTraj() {
        return doneWithTraj;
    }

    public enum DriveState {
        TELEOP_DRIVE,
        TELEOP_SPEAKER_TRACK,
        TELEOP_AMP_TRACK,
        LOCK,
        FOLLOW_TRAJ
    }
}