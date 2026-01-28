package frc.robot.visutils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.LimelightHelpers;
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
        System.out.println("MegatagOdometry: Limelight for pose estimates");
        System.out.println("---------------------------------------------------");
    }

    public void periodic() {

        if (doPoseEstimating) {
            addVisionMeasurementV1();
        }
    }

    private void addVisionMeasurementV1() {
        boolean doRejectUpdate = false;
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        if (mt1 == null) {
            // $TODO - I dont think this should ever be needed.
            // In simulation, limelight may not be present until a few cycles of periodic, since we
            // populate it via NetworkTables later.
            return;
        }

        if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
        {
            if(mt1.rawFiducials[0].ambiguity > .7)
            {
                doRejectUpdate = true;
            }
            if(mt1.rawFiducials[0].distToCamera > 3)
            {
                doRejectUpdate = true;
            }
        }
        if(mt1.tagCount == 0)
        {
            doRejectUpdate = true;
        }

        if(!doRejectUpdate)
        {
            Matrix<N3, N1> stdDevs = VecBuilder.fill(.5, .5, 9999999);
            estConsumer.accept(mt1.pose, mt1.timestampSeconds, stdDevs);
        }
    }

    // $TODO - Add this back
    // private void addVisionMeasurementV2() {
    //     boolean doRejectUpdate = false;
    //     LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    //     LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    //     if(Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    //     {
    //         doRejectUpdate = true;
    //     }
    //     if(mt2.tagCount == 0)
    //     {
    //         doRejectUpdate = true;
    //     }
    //     if(!doRejectUpdate)
    //     {
    //         m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
    //         m_poseEstimator.addVisionMeasurement(
    //             mt2.pose,
    //             mt2.timestampSeconds);
    //     }
    // }
}
