package frc.robot.visutils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.LimelightHelpers;
import java.util.Optional;
import frc.robot.sim.visionproducers.VisionSimInterface;
import static frc.robot.sim.visionproducers.VisionSimConstants.Vision.*;


public class LimelightOdometry {
    private Optional<Pose2d> latestVisPose = Optional.empty();

    /** Constructor */
    public LimelightOdometry(Field2d debugField) {
        this.debugField = debugField;
    }

    private VisionSimInterface.EstimateConsumer estConsumer;
    private final Field2d debugField; // $TODO - This should go away
    private Matrix<N3, N1> curStdDevs = kSingleTagStdDevs;
    private double lastTimestamp = 0;

    /**
     * Subscribe to pose estimates from this vision system.
     * @param consumer Lambda that will accept a pose estimate and pass it to your desired
     *     {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
     */
    public void subscribePoseEstimates(VisionSimInterface.EstimateConsumer consumer) {
        this.estConsumer = consumer;

        System.out.println("---------------------------------------------------");
        System.out.println("Limelight pose estimates subscribed\"");
        System.out.println("---------------------------------------------------");
    }

    public void periodic() {
        addVisionMeasurementV1();
    }

    private void addVisionMeasurementV1() {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        // Save the latest vision estimate so that it can be queried
        latestVisPose = Optional.ofNullable(mt1).map(est -> est.pose);

        if (mt1 == null) {
            // In simulation, limelight may not be present until a few cycles of periodic, since we
            // populate it via NetworkTables later.
            return;
        }

        // Skip if this is the same data we already processed
        if (mt1.timestampSeconds == lastTimestamp) {
            return;
        }
        lastTimestamp = mt1.timestampSeconds;

        // Update std devs based on tag count and distance
        updateEstimationStdDevs(mt1);

        // Check if we should reject this update
        if (mt1.tagCount == 0) {
            return;
        }

        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
            if (mt1.rawFiducials[0].ambiguity > 0.7) {
                return;
            }
        }

        // Check if std devs indicate rejection
        if (curStdDevs.get(0, 0) == Double.MAX_VALUE) {
            return;
        }

        // Print # of tags matching AND the stddevs values
        System.out.printf("LimelightOdometry: Adding vision measurement with %d tags, stdDevs=(%.2f, %.2f, %.2f)%n",
            mt1.tagCount, curStdDevs.get(0, 0), curStdDevs.get(1, 0), curStdDevs.get(2, 0));

        if (estConsumer != null) {
            estConsumer.accept(mt1.pose, mt1.timestampSeconds, curStdDevs);
        }

        // Add this point-in-time vision pose estimate to the debug field
        if (debugField != null) {
            //$TODO - I disabled this here since we print it in VisionSim
            //debugField.getObject("VisionEstimation").setPose(mt1.pose);
        }
    }

    /**
     * Calculates new standard deviations. This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags and distance from the tags.
     *
     * @param poseEstimate The Limelight pose estimate to evaluate
     */
    private void updateEstimationStdDevs(LimelightHelpers.PoseEstimate poseEstimate) {
        if (poseEstimate == null || poseEstimate.tagCount == 0) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;
            return;
        }

        // Pose present. Start running Heuristic
        var estStdDevs = kSingleTagStdDevs;
        int numTags = poseEstimate.tagCount;
        double avgDist = poseEstimate.avgTagDist;

        // One or more tags visible, run the full heuristic.
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
            estStdDevs = kMultiTagStdDevs;
        }

        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }

        curStdDevs = estStdDevs;
    }

    public Optional<Pose2d> getLatestVisPose() {
        return latestVisPose;
    }
}
