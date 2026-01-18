package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Constants for vision and odometry standard deviations used in pose estimation.
 * These values control how much the Kalman filter trusts odometry vs vision measurements.
 */
public class PhotonVisionSim {

    /**
     * Standard deviations for odometry measurements (x meters, y meters, theta radians).
     * Higher values = less trust in odometry, more reliance on vision.
     * These are set high for simulation to test vision correction.
     */
    public static final Matrix<N3, N1> kOdometryStandardDeviation = new Matrix<>(
        N3.instance, N1.instance, new double[] {
            0.1,   // x standard deviation (meters)
            0.1,   // y standard deviation (meters)
            0.1    // theta standard deviation (radians)
        }
    );

    /**
     * Standard deviations for vision measurements (x meters, y meters, theta radians).
     * Lower values = more trust in vision measurements.
     */
    public static final Matrix<N3, N1> kVisionStandardDeviation = new Matrix<>(
        N3.instance, N1.instance, new double[] {
            0.5,   // x standard deviation (meters)
            0.5,   // y standard deviation (meters)
            0.9    // theta standard deviation (radians)
        }
    );
}
