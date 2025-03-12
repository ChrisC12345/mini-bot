package team3647.frc2025.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import team3647.lib.PeriodicSubsystem;

public class Drivetrain implements PeriodicSubsystem {

    public static class PeriodicIO {
        //define inputs and outputs
        public double leftOutput;
        public double rightOutput;
    }

    private final SparkMax leftMotor = new SparkMax(4, null);
    private final SparkMax rightMotor = new SparkMax(5, null);
    private final PeriodicIO periodicIo = new PeriodicIO();

    // write drive method

    @Override
    public void writePeriodicOutputs() {
        // set periodicio leftoutput and rightoutput equal to a setreference thing
        leftMotor.getClosedLoopController().setReference(0, ControlType.kDutyCycle);
        rightMotor.getClosedLoopController().setReference(0, ControlType.kDutyCycle);
    }

    public void readPeriodicInputs(double forward, double rotation) {
        // call drive method       
        WheelSpeeds wheelspeeds = DifferentialDrive.arcadeDriveIK(forward, rotation, false);
        periodicIo.leftOutput = wheelspeeds.left;
        periodicIo.rightOutput = wheelspeeds.right;
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
