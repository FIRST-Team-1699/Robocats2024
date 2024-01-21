package frc.team1699.subsystems;

import java.io.File;
import java.io.IOException;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.team1699.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Drive {
    private DriveState currentState = DriveState.LOCK;
    private DriveState wantedState = DriveState.LOCK;
    private Timer trajTimer = new Timer();
    private PathPlannerTrajectory trajectory;
    private PPHolonomicDriveController driveController;
    private PIDConstants translationConstants = new PIDConstants(0.005 , 0, 0.0);
    private PIDConstants rotationConstants = new PIDConstants(0.01, 0, 0);
    private boolean doneWithTraj = true;

    private SwerveDirection targetDirection;

    private SwerveDrive swerve;
    private XboxController controller;
    public Drive(XboxController controller) {
        try {
            this.swerve = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(Units.feetToMeters(15.1));
        } catch (IOException e) {
            System.out.print("Swerve build failed");
        }
        this.controller = controller;
        this.driveController = new PPHolonomicDriveController(translationConstants, rotationConstants, SwerveConstants.kMaxSpeed, Units.inchesToMeters(14.5));
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    }

    private void teleopDrive() {
        // get controller inputs
        double vX = controller.getLeftX();
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
        vX *= SwerveConstants.kMaxSpeed*0.70; 
        vY *= SwerveConstants.kMaxSpeed*0.70;
        vR *= SwerveConstants.kMaxRotationalSpeed * 0.70 *0.70;

        // drive swerv
        swerve.drive(new Translation2d(vY, vX), vR, true, false);
    }

    private void headingControlledDrive() {
        double vX = -controller.getLeftX();
        double vY = -controller.getLeftY();
        
        // apply deadbands
        if(Math.abs(vX) < SwerveConstants.kDeadband) {
            vX = 0;
        }
        if(Math.abs(vY) < SwerveConstants.kDeadband) {
            vY = 0;
        }
        // scale outputs
        vX *= SwerveConstants.kMaxSpeed; 
        vY *= SwerveConstants.kMaxSpeed;

        // drive swerve
        swerve.driveFieldOriented(swerve.getSwerveController().getTargetSpeeds(vY, vX, targetDirection.getRadians(), swerve.getYaw().getRadians(), SwerveConstants.kMaxSpeed));
    }

    public void setTrajectory(PathPlannerTrajectory trajectory) {
        this.trajectory = trajectory;
    }

    // /** Manually set the module states
    //  * @param moduleStates
    //  * FL, FR, BL, BR
    //  */
    // private void setModuleStates(SwerveModuleState[] moduleStates) {
    //     swerve.setModuleStates(moduleStates, false);
    // }

    /** Set an X to keep the swerve from moving */
    private void lock() {
        swerve.lockPose();
    }

    public void resetHeading() {
        Rotation3d gyroReading = swerve.getGyroRotation3d();
        swerve.setGyro(new Rotation3d(gyroReading.getX(), gyroReading.getY(), 0.0));
    }

    public void update() {
        switch (currentState) {
            case FOLLOW_TRAJ:
                if(trajTimer.get() < trajectory.getTotalTimeSeconds()) {
                    PathPlannerTrajectory.State targetState = trajectory.sample(trajTimer.get());
                    ChassisSpeeds targetSpeeds = driveController.calculateRobotRelativeSpeeds(swerve.getPose(), targetState);
                    swerve.drive(targetSpeeds);
                } else {
                    trajTimer.stop();
                    doneWithTraj = true;
                    setWantedState(DriveState.LOCK);
                }
                break;
            case LOCK:
                lock();
                break;
            case TELEOP_DRIVE:
                teleopDrive();
                break;
            case TELEOP_HEADING_LOCKED:
                int pov = controller.getPOV();
                if(pov == -1) {
                    setWantedState(DriveState.TELEOP_DRIVE);
                } else {
                    targetDirection = new SwerveDirection(pov);
                    headingControlledDrive();
                }
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
                swerve.resetOdometry(new Pose2d(trajectory.sample(0).positionMeters, swerve.getYaw()));
                break;
            case LOCK:
                break;
            case TELEOP_DRIVE:
                break;
            case TELEOP_HEADING_LOCKED:
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
        return swerve.getYaw();
    }

    public boolean doneWithTraj() {
        return doneWithTraj;
    }

    private static class SwerveDirection {
        private double angle; 

        /** Takes an angle from 0-360 */
        public SwerveDirection(double angle) {
            this.angle = angle > 180.0 ? angle - 360.0 : angle;
        }

        public double getRadians() {
            return Rotation2d.fromDegrees(angle - 180).getRadians();
        }
    }

    public enum DriveState {
        TELEOP_DRIVE,
        TELEOP_HEADING_LOCKED,
        LOCK,
        FOLLOW_TRAJ
    }
}