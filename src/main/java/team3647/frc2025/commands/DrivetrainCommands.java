package team3647.frc2025.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;

import team3647.frc2025.subsystems.Drivetrain;

public class DrivetrainCommands {

    private final Drivetrain drivetrain;
    
    public DrivetrainCommands(Drivetrain dtrain){
        this.drivetrain = dtrain;
    }

    public Command drive(DoubleSupplier drive, DoubleSupplier turn){

        return Commands.run(
            () -> {
                SmartDashboard.putNumber("Drive", drive.getAsDouble());
                SmartDashboard.putNumber("Turn", turn.getAsDouble()); 
                SmartDashboard.putNumber("Left", drivetrain.getLeftMotorPosition());
                drivetrain.drive(drive.getAsDouble(),turn.getAsDouble());
            },drivetrain);
    }
 
    public Command turnWheel(){
        //drivetrain.setLeftMotorPosition();
        double setPoint = drivetrain.getLeftMotorPosition() + 1.25;
        SmartDashboard.putNumber("Left", drivetrain.getLeftMotorPosition());
        SmartDashboard.putNumber("Setpoint", setPoint);


        return Commands.run(
            () -> {
                SmartDashboard.putNumber("Setpoint", setPoint);
                SmartDashboard.putNumber("Left", drivetrain.getLeftMotorPosition());
                drivetrain.turn(setPoint);
            },
            drivetrain)
            .until(() -> (setPoint-0.2 < drivetrain.getLeftMotorPosition()) 
            && (drivetrain.getLeftMotorPosition() < setPoint+0.2));
    }
 
  /* 
    public class turnWheel extends Command {

        double motorPosition;

        @Override
        public void initialize(){
            this.motorPosition = drivetrain.getRightMotorPosition();
        }

        @Override
        public void execute(){
            drivetrain.turn(this.motorPosition);
        }

        @Override 
        public boolean isFinished(){
            return motorPosition-0.05 <= drivetrain.getRightMotorPosition() 
            && drivetrain.getRightMotorPosition() <= motorPosition+0.05;
        }
    } 
    public turnWheel turnWheel = new turnWheel();
  */
        
} 
