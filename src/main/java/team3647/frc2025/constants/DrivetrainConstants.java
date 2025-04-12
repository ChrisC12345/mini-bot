package team3647.frc2025.constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.ADIS16470_IMU;



public class DrivetrainConstants {

    public static final SparkMax leftMotor = new SparkMax(5, MotorType.kBrushless);
    public static final SparkMax rightMotor = new SparkMax(4, MotorType.kBrushless);

    public static final ADIS16470_IMU gyro = new ADIS16470_IMU();

    private static final SparkMaxConfig configLeft = new SparkMaxConfig();
    private static final SparkMaxConfig configRight = new SparkMaxConfig();


    static {
        configLeft.inverted(true);

        configLeft.idleMode(IdleMode.kBrake);
        configRight.idleMode(IdleMode.kBrake);

        configLeft.closedLoop.p(0.2);
        configLeft.closedLoop.i(0);
        configLeft.closedLoop.d(0);

        leftMotor.configure(configLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(configRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

}
