package frc.team1699.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.math.MathUtil;
import frc.team1699.Constants.PivoterConstants;

public class Pivoter {
    private CANSparkMax pivotMotor;
    private RelativeEncoder pivotEncoder;
    private SparkPIDController pivotController;
    private double setpoint;

    public Pivoter() {
        pivotMotor = new CANSparkMax(PivoterConstants.kMotorID, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getAlternateEncoder(Type.kQuadrature, 8192);
        pivotEncoder.setPositionConversionFactor(360);
        pivotMotor.setInverted(true);
        pivotEncoder.setInverted(true);
        pivotController = pivotMotor.getPIDController();
        pivotController.setFeedbackDevice(pivotEncoder);
        pivotController.setP(PivoterConstants.kP);
        pivotController.setI(PivoterConstants.kI);
        pivotController.setD(PivoterConstants.kD);
        pivotController.setFF(PivoterConstants.kFF);
        pivotController.setIZone(PivoterConstants.kIZone);
        pivotController.setOutputRange(-1, 1);
        pivotMotor.setSmartCurrentLimit(PivoterConstants.kPivotCurrentLimit);

        pivotController.setSmartMotionMaxVelocity(5700, 0);
        pivotController.setSmartMotionMaxAccel(5700, 0);
        pivotController.setSmartMotionAllowedClosedLoopError(PivoterConstants.kTolerance, 0);
    }

    // public void setAngle(double angle) {
    //     angle = MathUtil.clamp(angle, PivoterConstants.kMinAngle, PivoterConstants.kMaxAngle);
    //     setpoint = angle - 23;
    //     pivotController.setReference(setpoint, ControlType.kPosition);
    // }

    public void setAngle(double angle) {
        angle = MathUtil.clamp(angle, PivoterConstants.kMinAngle, PivoterConstants.kMaxAngle);
        setpoint = angle - 23;
        pivotController.setReference(setpoint, ControlType.kSmartMotion);
    }

    public boolean isAtAngle() {
        if(Math.abs(pivotEncoder.getPosition() - setpoint) <= PivoterConstants.kTolerance) {
            return true;
        }
        return false;
    }

    public void printCurrent() {
        System.out.println(pivotMotor.getOutputCurrent());
    }

    public void printPosition() {
        System.out.println(pivotEncoder.getPosition());
    }
}