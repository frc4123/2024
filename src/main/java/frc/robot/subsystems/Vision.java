package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{

    PhotonCamera camera = new PhotonCamera("photonvision");
     
    public Vision() {
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        List<PhotonTrackedTarget> targets = result.getTargets();
        PhotonTrackedTarget target = result.getBestTarget();

        // get data from AprilTag
        int targetID = target.getFiducialId();
        double poseAmbiguity = target.getPoseAmbiguity();
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

        // save to file 
        camera.takeInputSnapshot();
        camera.takeOutputSnapshot();

    }

    public void Periodic() {

    }


}
