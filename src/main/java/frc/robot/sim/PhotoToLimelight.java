package frc.robot.sim;

import static frc.robot.sim.VisionSimConstants.Vision.*;

import org.photonvision.PhotonCamera;
import frc.robot.Robot;

/**
 * Simpler vision processor that just prints tx/ty values from PhotonVision targets.
 */
public class PhotoToLimelight {
    private final PhotonCamera camera;

    public PhotoToLimelight() {
        if (!Robot.isSimulation()) {
            throw new IllegalStateException(
                "PhotoToLimelight should only be instantiated in simulation");
        }

        camera = new PhotonCamera(kCameraName);
    }

    public void periodic() {
        for (var result : camera.getAllUnreadResults()) {
            if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                    double tx = target.getYaw();
                    double ty = target.getPitch();
                    System.out.println("tx: " + tx + ", ty: " + ty);
                }
            }
        }
    }
}
