package frc.robot.commands.vision;

import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose3d;

import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.DetectedAlliance;
import frc.robot.Constants.VisionConstants;


public class VisionAllign extends Command{
    Vision vision;

    public VisionAllign(){
        
    }

    // Pose3d cameraRobotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()), cameraToRobot);
    void periodic() {
        SmartDashboard.putString("Detected Alliance: ", vision.getAllianceStatus() == DetectedAlliance.BLUE ? "BLUE"
                : vision.getAllianceStatus() == DetectedAlliance.RED ? "RED" : "None");
    }


}
