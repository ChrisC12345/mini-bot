package team3647.frc2025.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import team3647.lib.PeriodicSubsystem;

public class Drivetrain implements PeriodicSubsystem {

    public static class PeriodicIO {
        //define inputs and outputs
        public double leftOutput;
        public double rightOutput;
    }

    private final SparkMax leftMotor = new SparkMax(4, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(5, MotorType.kBrushless);
    private final PIDController pid = new PIDController(0.1, 0, 0);
    private final PeriodicIO periodicIo = new PeriodicIO();

    // write drive method

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
        // turn wheel 90 degrees
        periodicIo.rightOutput = pid.calculate(rightMotor.getEncoder().getPosition(), setPoint);
    }

    public double getRightMotorPosition(){
        return rightMotor.getEncoder().getPosition();
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
