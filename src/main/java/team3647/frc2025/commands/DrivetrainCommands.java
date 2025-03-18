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
                drivetrain.drive(drive.getAsDouble(),turn.getAsDouble());
            },drivetrain);
    }
 
    public Command turnWheel(){
        
        double setPoint = drivetrain.getRightMotorPosition()+1.25;
        SmartDashboard.putNumber("Setpoint", drivetrain.getRightMotorPosition()+1.25);

        return Commands.run(
            () -> {SmartDashboard.putNumber("Right", drivetrain.getRightMotorPosition());
                drivetrain.turn(setPoint);},
            drivetrain)
            .until(() -> setPoint-0.05 <= drivetrain.getRightMotorPosition() 
            && drivetrain.getRightMotorPosition() <= setPoint+0.05);
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
