package frc.team1699.lib.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * A PathPlannerPath wrapper to handle flipping trajectories for red alliance.
 */
public class HandledPath {
    private PathPlannerPath path;
    private Rotation2d startingRotation;
    private ChassisSpeeds startingSpeeds;
    
    /**
     * Starting trajectory rotation and speeds are zero.
     * @param pathName
     * Name of the path file
     */
    public HandledPath(String pathName) {
        this.path = PathPlannerPath.fromPathFile(pathName);
        this.startingRotation = new Rotation2d();
        this.startingSpeeds = new ChassisSpeeds();
    }

    /**
     * Starting trajectory speeds are zero.
     * @param pathName
     * Name of the path file
     * @param startingRotation
     * Initial trajectory heading
     */
    public HandledPath(String pathName, Rotation2d startingRotation) {
        this.path = PathPlannerPath.fromPathFile(pathName);
        this.startingRotation = startingRotation;
        this.startingSpeeds = new ChassisSpeeds();
    }

    /**
     * @param pathName
     * Name of the path file
     * @param startingRotation
     * Initial trajectory heading
     * @param startingSpeeds
     * Initial trajectory speeds
     */
    public HandledPath(String pathName, Rotation2d startingRotation, ChassisSpeeds startingSpeeds) {
        this.path = PathPlannerPath.fromPathFile(pathName);
        this.startingRotation = startingRotation;
        this.startingSpeeds = startingSpeeds;
    } 

    /**
     * 
     * @return
     * A generated trajectory, flipped based on alliance.
     */
    public PathPlannerTrajectory getTrajectory() {
        return DriverStation.getAlliance().get() == Alliance.Red ? 
            path.flipPath().getTrajectory(startingSpeeds, Rotation2d.fromDegrees(startingRotation.getDegrees() * -1)) : 
            path.getTrajectory(startingSpeeds, startingRotation);
    }
}
