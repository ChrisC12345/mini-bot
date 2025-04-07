package team3647.frc2025.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

import team3647.lib.PeriodicSubsystem;
import team3647.frc2025.robot.LimelightHelpers;


@SuppressWarnings("unused")
public class Drivetrain implements PeriodicSubsystem {
    
    public static class PeriodicIO {
        //define inputs and outputs
        public double leftOutput;
        public double rightOutput;
        ControlType controltype = ControlType.kPosition;
    }

        
    public Drivetrain(SparkMax left, SparkMax right, ADIS16470_IMU gyro){
        robotPose = new Pose2d(0,0,new Rotation2d());
        leftMotor = left;
        rightMotor = right;
        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);
        this.gyro = gyro;

        this.poseEstimator = 
            new DifferentialDrivePoseEstimator(
                kinematics, Rotation2d.fromDegrees(gyro.getAngle()), 
                0.0,0.0,
                robotPose);
    }

    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final ADIS16470_IMU gyro;
    private final DifferentialDrivePoseEstimator poseEstimator;
    private final PIDController pid = new PIDController(0.08, 0, 0);
    private final PeriodicIO periodicIo = new PeriodicIO();
    private Pose2d robotPose;
    

    private final DifferentialDriveKinematics kinematics = 
        new DifferentialDriveKinematics(Units.inchesToMeters(14.0));

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
            SmartDashboard.putNumber("LeftMotorPosition", getLeftMotorPosition());
            SmartDashboard.putNumber("RightMotorPosition", getRightMotorPosition());
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

            LimelightHelpers.PoseEstimate limelightMesaurement = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7,0.7,9999999));
            poseEstimator.addVisionMeasurement(
                limelightMesaurement.pose,
                limelightMesaurement.timestampSeconds);

            robotPose = poseEstimator.update(
                Rotation2d.fromDegrees(gyro.getAngle()),
                leftMotor.getEncoder().getPosition()*0.1*Math.PI*0.2,
                rightMotor.getEncoder().getPosition()*0.1*Math.PI*0.2);

            Logger.recordOutput("Pose", robotPose);
    } 

    @Override
    public String getName() {
        return "drivetrain";
    }
    
}
