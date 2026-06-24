package team3647.frc2025.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import team3647.frc2025.constants.DrivetrainConstants;
import team3647.lib.LimelightHelpers;
import team3647.lib.PeriodicSubsystem;

@SuppressWarnings("unused")
public class Drivetrain implements PeriodicSubsystem {

    public static class PeriodicIO {
        public double leftOutput;
        public double rightOutput;
        public ControlType controlType = ControlType.kPosition;
        public Pose2d pose = new Pose2d();
    }

    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final ADIS16470_IMU gyro;
    private final DifferentialDrivePoseEstimator poseEstimator;
    private final PIDController pid = new PIDController(0.08, 0, 0);
    private final PeriodicIO periodicIO = new PeriodicIO();

    public Drivetrain(SparkMax left, SparkMax right, ADIS16470_IMU gyro) {
        leftMotor = left;
        rightMotor = right;
        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);
        this.gyro = gyro;

        poseEstimator = new DifferentialDrivePoseEstimator(
            DrivetrainConstants.KINEMATICS,
            getHeading(),
            0.0, 0.0,
            new Pose2d());

        LimelightHelpers.setCameraPose_RobotSpace(
            "limelight",
            Units.inchesToMeters(7), Units.inchesToMeters(4), Units.inchesToMeters(9),
            0, 0, 180);
    }

    // --- Sensors ---

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    private double getLeftDistanceMeters() {
        return (leftMotor.getEncoder().getPosition() / DrivetrainConstants.GEAR_RATIO)
            * 2 * Math.PI * DrivetrainConstants.WHEEL_RADIUS_METERS;
    }

    private double getRightDistanceMeters() {
        return (rightMotor.getEncoder().getPosition() / DrivetrainConstants.GEAR_RATIO)
            * 2 * Math.PI * DrivetrainConstants.WHEEL_RADIUS_METERS;
    }

    public double getLeftMotorPosition() {
        return leftMotor.getEncoder().getPosition();
    }

    public double getRightMotorPosition() {
        return rightMotor.getEncoder().getPosition();
    }

    public Pose2d getPose() {
        return periodicIO.pose;
    }

    // --- Drive methods ---

    public void drive(double forward, double rotation) {
        WheelSpeeds wheelspeeds = DifferentialDrive.arcadeDriveIK(forward, -rotation, false);
        periodicIO.controlType = ControlType.kDutyCycle;
        periodicIO.leftOutput = wheelspeeds.left;
        periodicIO.rightOutput = wheelspeeds.right;
    }

    // Method 2: SparkMax onboard PID at 1000Hz
    public void turn(double setPoint) {
        periodicIO.controlType = ControlType.kPosition;
        periodicIO.leftOutput = setPoint;
    }

    // Method 1: WPILib PIDController at 50Hz
    public void setAngle(double targetRotations) {
        periodicIO.controlType = ControlType.kDutyCycle;
        periodicIO.leftOutput = pid.calculate(getLeftMotorPosition(), targetRotations);
    }

    public double degreesToRotations(double degrees) {
        return (degrees / 360.0) * DrivetrainConstants.GEAR_RATIO;
    }

    public void setLeftEncoderPosition(double position) {
        leftMotor.getEncoder().setPosition(position);
    }

    // --- Periodic ---

    @Override
    public void readPeriodicInputs() {
        // Update odometry
        periodicIO.pose = poseEstimator.update(
            getHeading(),
            getLeftDistanceMeters(),
            getRightDistanceMeters());

        // Add vision measurement if valid
        LimelightHelpers.PoseEstimate limelightMeasurement =
            LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        if (limelightMeasurement != null && !limelightMeasurement.pose.equals(Pose2d.kZero)) {
            var tags = LimelightHelpers.getRawFiducials("limelight");
            double ambiguity = tags.length > 0 ? tags[0].ambiguity : 1.0;
            double distance = tags.length > 0 ? tags[0].distToCamera : 9999;

            if (ambiguity < 0.2) {
                poseEstimator.setVisionMeasurementStdDevs(
                    VecBuilder.fill(0.05, 0.05, 9999999).times(distance * 0.01));
                // timestampSeconds is already latency-corrected by LimelightHelpers
                poseEstimator.addVisionMeasurement(
                    limelightMeasurement.pose,
                    limelightMeasurement.timestampSeconds);
            }
        }
    }

    @Override
    public void writePeriodicOutputs() {
        leftMotor.getClosedLoopController().setReference(periodicIO.leftOutput, periodicIO.controlType);
        rightMotor.getClosedLoopController().setReference(periodicIO.rightOutput, periodicIO.controlType);

        Logger.recordOutput("Drivetrain/Pose", periodicIO.pose);
        Logger.recordOutput("Drivetrain/LeftDistanceMeters", getLeftDistanceMeters());
        Logger.recordOutput("Drivetrain/RightDistanceMeters", getRightDistanceMeters());
        Logger.recordOutput("Drivetrain/HeadingDegrees", gyro.getAngle());
    }

    @Override
    public String getName() {
        return "Drivetrain";
    }
}
