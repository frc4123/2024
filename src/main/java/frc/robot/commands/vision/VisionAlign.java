package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.DetectedAlliance;


public class VisionAlign extends Command{
    Vision vision;
    Pose3d visionPose;

    public void VisionAllign(){
        
    }

    // Pose3d cameraRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()), cameraToRobot);
    void periodic(Vision vision) {
        visionPose = vision.get3dPose();
        SmartDashboard.putString("Detected Alliance: ", vision.getAllianceStatus() == DetectedAlliance.BLUE ? "BLUE"
                : vision.getAllianceStatus() == DetectedAlliance.RED ? "RED" : "None");
    }


}
