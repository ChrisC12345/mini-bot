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
    
        return new Command() {

        double setPoint;
        
        @Override
        public void initialize(){
            setPoint = drivetrain.getLeftMotorPosition()+1.25;
        }

        @Override
        public void execute(){
            drivetrain.turn(setPoint);
        }

        @Override 
        public boolean isFinished(){
            return setPoint-0.2 <= drivetrain.getLeftMotorPosition() 
            && drivetrain.getLeftMotorPosition() <= setPoint+0.2;
        }
    };
} 

 /* 
    public Command turnWheel(){
        //drivetrain.setLeftMotorPosition();
        double starting = drivetrain.getLeftMotorPosition();
        SmartDashboard.putNumber("Left", drivetrain.getLeftMotorPosition());
        SmartDashboard.putNumber("SetpointInit", starting+5.0);


        return Commands.run(
            () -> {
                SmartDashboard.putNumber("SetpointPeriodic", starting+5.0);
                SmartDashboard.putNumber("Left", drivetrain.getLeftMotorPosition());
                drivetrain.turn(starting+0.5);
            },
            drivetrain)
            .until(() -> (starting+0.5-0.2 < drivetrain.getLeftMotorPosition()) 
            && (drivetrain.getLeftMotorPosition() < starting+5.0+0.2))
            .finallyDo(()->SmartDashboard.putNumber("Command End",starting+5.0));
    }
*/


} 
