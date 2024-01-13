package frc.robot.team1699.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.team1699.Constants;

public class Drivetrain {
    private TalonFX portLeader;
    private TalonFX portFollower;
    private TalonFX starLeader;
    private TalonFX starFollower;

    public Drivetrain() {
        this.portLeader = new TalonFX(Constants.kPortLeaderID);
        this.portFollower = new TalonFX(Constants.kPortFollowerID);
        portFollower.follow(portLeader, FollowerType.PercentOutput);

        this.starLeader = new TalonFX(Constants.kStarLeaderID);
        this.starFollower = new TalonFX(Constants.kStarFollowerID);
        starFollower.follow(starLeader, FollowerType.PercentOutput);
    }

    public void runArcadeDrive(double throttle, double rotate) {
        double portOutput = 0.0;
        double starOutput = 0.0;

        if(Math.abs(throttle) < .1) {
            throttle = 0;
        }
        if(Math.abs(rotate) < .1) {
            rotate = 0;
        }

        throttle = Math.copySign(throttle * throttle, throttle);
        rotate = Math.copySign(rotate * rotate, rotate);

        double maxInput = Math.copySign(Math.max(Math.abs(throttle), Math.abs(rotate)), throttle);

        if (throttle >= 0.0) {
            // First quadrant, else second quadrant
            if (rotate >= 0.0) {
                portOutput = maxInput;
                starOutput = throttle - rotate;
            } else {
                portOutput = throttle + rotate;
                starOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (rotate >= 0.0) {
                portOutput = throttle + rotate;
                starOutput = maxInput;
            } else {
                portOutput = maxInput;
                starOutput = throttle - rotate;
            }
        }

        portLeader.set(TalonFXControlMode.PercentOutput, -portOutput);
        starLeader.set(TalonFXControlMode.PercentOutput, starOutput);
    }

    public void setIdleMode(NeutralMode idleMode) {
        portLeader.setNeutralMode(idleMode);
        portFollower.setNeutralMode(idleMode);
        starLeader.setNeutralMode(idleMode);
        starFollower.setNeutralMode(idleMode);
    }
}