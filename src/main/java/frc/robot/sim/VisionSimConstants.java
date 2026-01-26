package frc.robot.sim;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Robot;
import java.util.ArrayList;

public class VisionSimConstants {
    public static class Vision {
        // Camera name (must match PhotonVision camera name)
        public static final String kCameraName = "photonvision";

        // Camera position relative to robot center
        // Example: mounted facing forward, 0.5m forward of center, 0.5m up from center
        public static final Transform3d kRobotToCam = new Transform3d(
            new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(0, 0, 0)
        );

        // The layout of the AprilTags on the field
        public static AprilTagFieldLayout kTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Static initializer to add custom calibration tags in simulation
        static {
            if (Robot.isSimulation()) {
                kTagLayout = addCustomVisionCalibrationTags(kTagLayout);
            }
        }

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        /**
         * Adds custom calibration tags to the provided field layout for vision testing.
         * Currently adds tag 101 at field center for testing field alignment.
         * @param baseLayout The field layout to add custom tags to
         * @return A new AprilTagFieldLayout with the custom tags added
         */
        // $TODO - Eventually move this into the utility we use to calibrate drivetrain using custom
        // vision tags.
        public static AprilTagFieldLayout addCustomVisionCalibrationTags(AprilTagFieldLayout baseLayout) {
            if (!Robot.isSimulation()) {
                throw new IllegalStateException("addCustomVisionCalibrationTags should only be called in simulation");
            }

            // Create a mutable list with all existing tags
            var tags = new ArrayList<>(baseLayout.getTags());

            // Add custom tag 101 at field center (16.541m x 8.069m field)
            // Placed at center, 0.5m height, facing toward blue alliance (+X direction)
            tags.add(new AprilTag(101, new Pose3d(
                8.2705,  // x: center of field length
                4.0345,  // y: center of field width
                0.5,     // z: 0.5m height
                new Rotation3d(0, 0, 0)  // facing +X (toward blue alliance)
            )));

            return new AprilTagFieldLayout(tags, baseLayout.getFieldLength(), baseLayout.getFieldWidth());
        }
    }
}
