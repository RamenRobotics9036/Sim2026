package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Constants for vision and odometry standard deviations used in pose estimation.
 * These values control how much the Kalman filter trusts odometry vs vision measurements.
 */
public class PhotonVisionSim {

    /**
     * Constructs a PhotonVisionSim instance.
     * This class is only intended for use in simulation.
     *
     * @throws IllegalStateException if called outside of simulation mode
     */
    public PhotonVisionSim() {
        if (!Robot.isSimulation()) {
            throw new IllegalStateException("PhotonVisionSim should only be instantiated in simulation mode");
        }
    }

    /**
     * Standard deviations for vision measurements (x meters, y meters, theta radians).
     * Lower values = more trust in vision measurements.
     *
     * Note that the DEFAULT for VISION is [0.9, 0.9, 0.9] - which means dont trust vision too much.
     * The default for ODOMETRY is [0.1, 0.1, 0.1] - which means trust odometry a lot.
     */
    public static final Matrix<N3, N1> kVisionStandardDeviation = new Matrix<>(
        N3.instance, N1.instance, new double[] {
            0.5,   // x standard deviation (meters)
            0.5,   // y standard deviation (meters)
            0.9    // theta standard deviation (radians)
        }
    );

    /**
     * Configures the drivetrain to trust vision measurements more than the default.
     * This sets lower standard deviations for vision, making the Kalman filter
     * weight vision corrections more heavily.
     *
     * @param drivetrain The swerve drivetrain to configure
     */
    public void setDrivetrainToTrustVisionMore(SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain) {
        drivetrain.setVisionMeasurementStdDevs(PhotonVisionSim.kVisionStandardDeviation);
    }
}
