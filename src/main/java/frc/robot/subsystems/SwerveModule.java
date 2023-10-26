package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule{
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed,
        boolean turningMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed){

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        // we don't need the integral or derivative for the angular motor
        turningPidController = new PIDController(ModuleConstants.kPTurning,0,0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI); // Tells PID that the wheel  is circular so that it can jump through the gap of 180 degrees
    }

    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }

    public double getTurningPosition(){
        return turningEncoder.getPosition();

    }

    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity(){
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad(){
        /*dividing the voltage reading from the 
        encoder by the voltage supplied to the encoder
        to get the angle in %. Then we multiply it by
        2 pi to convert it to radians*/ 
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0*Math.PI;
        // Converting relative angle to absolute angle
        angle -= absoluteEncoderOffsetRad;
        //returning reversed angles if the absolute encoder is reversed
        return angle*(absoluteEncoderReversed ? -1.0:1.0);
        //motors will lose readings when they restart
        //That is why we will need to reset motors
    }

    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        // Adding an if statement to ensure that wpilib does not reset motor positions if we do not want it to
        if (Math.abs(state.speedMetersPerSecond)< 0.001){
            stop();
            return;
        }
        // Making to motor not have to move more than 90 degrees
        state = SwerveModuleState.optimize(state, getState().angle);
        //scaling velocity down using the robot's max speed. Setting speed to drive motor
        driveMotor.set(state.speedMetersPerSecond/DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        //using the setpoint and PID controller to set angle to the turning motor
        turningMotor.set(turningPidController.calculate(getTurningPosition(),state.angle.getRadians()));
        //sending speed and angle to the dashboard
        SmartDashboard.putString("Swerve["+absoluteEncoder.getChannel()+ "]",state.toString());
    }

    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }

}
