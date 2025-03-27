package team3647.frc2025.subsystems;

import com.revrobotics.spark.SparkMax;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;

import team3647.lib.PeriodicSubsystem;
import team3647.frc2025.constants.DrivetrainConstants;


public class Drivetrain implements PeriodicSubsystem {
    
    public static class PeriodicIO {
        //define inputs and outputs
        public double leftOutput;
        public double rightOutput;
        ControlType controltype = ControlType.kPosition;
    }

    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final ADIS16470_IMU gyro;
    DifferentialDriveOdometry odometry;
    private final PIDController pid = new PIDController(0.08, 0, 0);
    private final PeriodicIO periodicIo = new PeriodicIO();
    Pose2d robotPose;

    public Drivetrain(SparkMax left, SparkMax right, ADIS16470_IMU gyro){
        robotPose = new Pose2d();
        leftMotor = left;
        rightMotor = right;
        this.gyro = gyro;
        this.odometry = new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(gyro.getAngle()), 
            leftMotor.getEncoder().getPosition(), 
            rightMotor.getEncoder().getPosition(),
            new Pose2d(0,0,new Rotation2d()));
    }

    // write drive method

    @Override 
    public void readPeriodicInputs(){
        Logger.recordOutput("Pose",robotPose);
    }

    @Override
    public void writePeriodicOutputs() {
        // set periodicio leftoutput and rightoutput equal to a setreference thing
        leftMotor.getClosedLoopController().setReference(periodicIo.leftOutput, periodicIo.controltype);
        rightMotor.getClosedLoopController().setReference(periodicIo.rightOutput, periodicIo.controltype);
    }

    public void drive(double forward, double rotation) {
        // call drive method 
        WheelSpeeds wheelspeeds = DifferentialDrive.arcadeDriveIK(forward, rotation, false);
        periodicIo.controltype = ControlType.kDutyCycle;
        periodicIo.leftOutput = wheelspeeds.left;
        periodicIo.rightOutput = wheelspeeds.right;
    }

    public void turn(double setPoint){
        // turn wheel to setPoint

        periodicIo.leftOutput = setPoint;
        periodicIo.controltype = ControlType.kPosition;
        SmartDashboard.putNumber("PID Calc", periodicIo.leftOutput);
    }
    
    public double getLeftMotorPosition(){
        return leftMotor.getEncoder().getPosition();
    }

    public double getRightMotorPosition(){
        return rightMotor.getEncoder().getPosition();
    }

    public void setLeftMotorPosition(){
        leftMotor.getEncoder().setPosition(0);

    }

    @Override
    public void periodic() {
        readPeriodicInputs();
        writePeriodicOutputs();

        robotPose = odometry.update(
            Rotation2d.fromDegrees(gyro.getAngle()),
            leftMotor.getEncoder().getPosition(),
            rightMotor.getEncoder().getPosition());
    } 

    @Override
    public String getName() {
        return "drivetrain";
    }
    
}
