package frc.robot.visutils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.sim.VisionSimInterface;


public class MegatagOdometry {

    private boolean doPoseEstimating = false;
    private VisionSimInterface.EstimateConsumer estConsumer;

    /**
     * Subscribe to pose estimates from this vision system.
     * @param consumer Lambda that will accept a pose estimate and pass it to your desired
     *     {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
     */
    public void subscribePoseEstimates(VisionSimInterface.EstimateConsumer consumer) {
        this.estConsumer = consumer;

        // Someone is subscribed to pose estimates - so lets start generating them
        doPoseEstimating = true;

        System.out.println("---------------------------------------------------");
        System.out.println("VMegatagOdometry: Limelight for pose estimates");
        System.out.println("---------------------------------------------------");
    }

    private void checkAndAddVisionMeasurement() {
        if (doPoseEstimating) {
            // For now, just create 2d pose at origin.  stdDevs is hardcoded to medium confidence.
            Matrix<N3, N1> stdDevs = VecBuilder.fill(0.15, 0.15, 3.0); // x, y, heading
            estConsumer.accept(new edu.wpi.first.math.geometry.Pose2d(), edu.wpi.first.wpilibj.Timer.getFPGATimestamp(), stdDevs);
        }
    }

    public void periodic() {
        // We only do pose estimation if someone is subscribed
        checkAndAddVisionMeasurement();
    }
}
