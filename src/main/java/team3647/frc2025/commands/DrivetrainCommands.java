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
                drivetrain.readPeriodicInputs(drive.getAsDouble(),turn.getAsDouble());
            },
            drivetrain);
    }
}
