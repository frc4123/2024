package frc.robot.subsystems;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {

    PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
    static final Set<Integer> redTargets = new HashSet<>(Arrays.asList(3, 4, 5, 9, 10, 11, 12, 13));
    static final Set<Integer> blueTargets = new HashSet<>(Arrays.asList(1, 2, 6, 7, 8, 14, 15, 16));
    public AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();    
    public enum DetectedAlliance {RED,BLUE};

    public DetectedAlliance getAllianceStatus() {
        var result = camera.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        var redTargetCount = 0;
        var blueTargetCount = 0;

        for (PhotonTrackedTarget target : targets) {
            if (redTargets.contains(target.getFiducialId())) {
                redTargetCount += 1;
            }
            if (blueTargets.contains(target.getFiducialId())) {
                blueTargetCount += 1;
            }
        }

        if (redTargetCount > blueTargetCount && redTargetCount >= VisionConstants.DETECTED_ALLIANCE_TRHESHOLD) {
            return DetectedAlliance.RED;
        } else if (blueTargetCount > redTargetCount && blueTargetCount >= VisionConstants.DETECTED_ALLIANCE_TRHESHOLD) {
            return DetectedAlliance.BLUE;
        } else return null;
    }

    public Pose3d get3dPose() {
        var result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        Optional<Pose3d> optionalPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());

        Pose3d cameraRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), optionalPose.get(), VisionConstants.cameraToRobot);
        return(cameraRobotPose);
    }

    public boolean hasTarget() {
        var result = camera.getLatestResult();
        return result.hasTargets();
    }

    public double getCamTimeStamp() {
        var imageCaptureTime = camera.getLatestResult().getTimestampSeconds();
        return imageCaptureTime;
    }
    
    @Override
    public void periodic() {
    }

}
