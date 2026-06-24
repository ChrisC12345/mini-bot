package team3647.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;

import team3647.frc2025.subsystems.Drivetrain;

public class DrivetrainCommands {

    private final Drivetrain drivetrain;

    public DrivetrainCommands(Drivetrain dtrain) {
        this.drivetrain = dtrain;
    }

    public Command drive(DoubleSupplier drive, DoubleSupplier turn) {
        return Commands.run(
            () -> drivetrain.drive(drive.getAsDouble(), turn.getAsDouble()),
            drivetrain);
    }

    // Method 2: SparkMax onboard PID — target is passed as motor rotations,
    // the SparkMax runs its internal PID loop at 1000Hz to reach it
    public Command turnWheel() {
        return new Command() {
            double setPoint;

            @Override
            public void initialize() {
                setPoint = drivetrain.getLeftMotorPosition() + 1.25;
            }

            @Override
            public void execute() {
                drivetrain.turn(setPoint);
            }

            @Override
            public boolean isFinished() {
                return Math.abs(drivetrain.getLeftMotorPosition() - setPoint) <= 0.05;
            }
        };
    }

    // Method 1: WPILib PIDController — pid.calculate() runs in robot code at 50Hz
    // and outputs a duty cycle value sent to the motor each loop
    public Command turnWheelPID() {
        return new Command() {
            double setPoint;

            @Override
            public void initialize() {
                setPoint = drivetrain.getLeftMotorPosition() + drivetrain.degreesToRotations(90);
            }

            @Override
            public void execute() {
                drivetrain.setAngle(setPoint);
            }

            @Override
            public boolean isFinished() {
                return Math.abs(drivetrain.getLeftMotorPosition() - setPoint) <= 0.05;
            }
        };
    }
}
