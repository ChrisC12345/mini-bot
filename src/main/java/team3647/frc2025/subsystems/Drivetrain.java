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
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team3647.lib.PeriodicSubsystem;



public class Drivetrain implements PeriodicSubsystem {
    
    public static class PeriodicIO {
        //define inputs and outputs
        public double leftOutput;
        public double rightOutput;
    }

    public Drivetrain(){
        robotPose = new Pose2d();
    }

    private final SparkMax leftMotor = new SparkMax(4, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(5, MotorType.kBrushless);
    private final PIDController pid = new PIDController(0.08, 0, 0);
    private final PeriodicIO periodicIo = new PeriodicIO();
    Pose2d robotPose;

    public class Odometry extends VirtualSubsystem{
        
        public Odometry(Pose3d cameraToRobot, RelativeEncoder leftEncoder, RelativeEncoder rightEncoder){
            poseEstimator = new DifferentialDrivePoseEstimator(
                new 
            )
        }
    }

    // write drive method

    @Override 
    public void readPeriodicInputs(){
        Logger.recordOutput("Pose",robotPose);
    }

    @Override
    public void writePeriodicOutputs() {
        // set periodicio leftoutput and rightoutput equal to a setreference thing
        leftMotor.getClosedLoopController().setReference(periodicIo.leftOutput, ControlType.kDutyCycle);
        rightMotor.getClosedLoopController().setReference(periodicIo.rightOutput, ControlType.kDutyCycle);
    }

    public void drive(double forward, double rotation) {
        // call drive method 
        WheelSpeeds wheelspeeds = DifferentialDrive.arcadeDriveIK(forward, rotation, false);
        periodicIo.leftOutput = wheelspeeds.left;
        periodicIo.rightOutput = wheelspeeds.right;
    }

    public void turn(double setPoint){
        // turn wheel to setPoint
        periodicIo.leftOutput = pid.calculate(leftMotor.getEncoder().getPosition(), setPoint);
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
    } 

    @Override
    public String getName() {
        return "drivetrain";
    }
    
}
