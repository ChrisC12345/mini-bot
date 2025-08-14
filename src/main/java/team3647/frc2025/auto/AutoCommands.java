package team3647.frc2025.auto;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import team3647.frc2025.constants.AutoConstants;
import team3647.frc2025.subsystems.Drivetrain;



public class AutoCommands {
   

        Alliance color;
        boolean isRed = true;
        Drivetrain dtrain;
        Supplier<Twist2d> autoDriveVelocities;
        BooleanSupplier hasTarget;

        public AutoCommands(
            Drivetrain dtrain,
            Supplier<Twist2d> autoDriveVelocities,
            BooleanSupplier hasTarget) {
        color = Alliance.Red;

        this.dtrain = dtrain;
        this.autoDriveVelocities = autoDriveVelocities;
        this.hasTarget = hasTarget;
        }

        final PIDController rotController = new PIDController(5, 0, 0);

    
}
