package team3647.frc2025.constants;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;

public class AutoConstants {

    public static SwerveSample kEmptySample =
            new SwerveSample(-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, new double[] {0}, new double[] {0});


    public static final PIDController rotController = new PIDController(4, 0, 0);
    public static final PIDController autoXController = new PIDController(4, 0, 0.1);
    public static final PIDController autoYController = new PIDController(4, 0, 0.1);

}
