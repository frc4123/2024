// // package frc.robot.subsystems;

// // import org.photonvision.PhotonCamera;
// // import org.photonvision.PhotonPoseEstimator;
// // import org.photonvision.PhotonPoseEstimator.PoseStrategy;

// // import edu.wpi.first.apriltag.AprilTagFieldLayout;
// // import edu.wpi.first.apriltag.AprilTagFields;
// // import edu.wpi.first.math.geometry.Rotation3d;
// // import edu.wpi.first.math.geometry.Transform3d;
// // import edu.wpi.first.math.geometry.Translation3d;
// // import frc.robot.Constants;

// // public class Vision {
// //     Transform3d robotToCam = new Transform3d(new Translation3d(-.264, 0.0, -0.17), new Rotation3d(0,0,6.28318531)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
// //     private final AprilTagFieldLayout aprilTagFieldLayout=AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
// //     private PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
// //     public final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);


// package frc.robot.subsystems;

// import static frc.robot.Constants.Vision.*;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import java.util.Optional;
// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.simulation.PhotonCameraSim;
// import org.photonvision.simulation.SimCameraProperties;
// import org.photonvision.simulation.VisionSystemSim;
// import org.photonvision.targeting.PhotonPipelineResult;
// import swervelib.telemetry.SwerveDriveTelemetry;
// import java.io.IOException;
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;

// public class Vision {
//     private final PhotonCamera camera;
//     private final PhotonPoseEstimator photonEstimator;
//     private double lastEstTimestamp = 0;

//     // Simulation
//     private PhotonCameraSim cameraSim;
//     private VisionSystemSim visionSim;
//     public AprilTagFieldLayout kTagLayout;

//     public Vision() {
//         camera = new PhotonCamera(kCameraName);

//         kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

//         photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, kRobotToCam);
//         photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

//         // ----- Simulation
//         if (SwerveDriveTelemetry.isSimulation) {
//             // Create the vision system simulation which handles cameras and targets on the field.
//             visionSim = new VisionSystemSim("main");
//             // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
//             visionSim.addAprilTags(kTagLayout);
//             // Create simulated camera properties. These can be set to mimic your actual camera.
//             var cameraProp = new SimCameraProperties();
//             cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
//             cameraProp.setCalibError(0.35, 0.10);
//             cameraProp.setFPS(15);
//             cameraProp.setAvgLatencyMs(50);
//             cameraProp.setLatencyStdDevMs(15);
//             // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
//             // targets.
//             cameraSim = new PhotonCameraSim(camera, cameraProp);
//             // Add the simulated camera to view the targets on this simulated field.
//             visionSim.addCamera(cameraSim, kRobotToCam);

//             cameraSim.enableDrawWireframe(true);
//         }
//     }

//     public PhotonPipelineResult getLatestResult() {
//         return camera.getLatestResult();
//     }

//     /**
//      * The latest estimated robot pose on the field from vision data. This may be empty. This should
//      * only be called once per loop.
//      *
//      * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
//      *     used for estimation.
//      */
//     public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
//         //photonEstimator.setReferencePose(prevEstimatedRobotPose);
//         var visionEst = photonEstimator.update();
//         double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
//         boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
//         if (SwerveDriveTelemetry.isSimulation) {
//             visionEst.ifPresentOrElse(
//                     est ->
//                             getSimDebugField()
//                                     .getObject("VisionEstimation")
//                                     .setPose(est.estimatedPose.toPose2d()),
//                     () -> {
//                         if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
//                     });
//         }
//         if (newResult) lastEstTimestamp = latestTimestamp;
//         return visionEst;
//     }

//     /**
//      * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
//      * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
//      * This should only be used when there are targets visible.
//      *
//      * @param estimatedPose The estimated pose to guess standard deviations for.
//      */
//     public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
//         var estStdDevs = kSingleTagStdDevs;
//         var targets = getLatestResult().getTargets();
//         int numTags = 0;
//         double avgDist = 0;
//         for (var tgt : targets) {
//             var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
//             if (tagPose.isEmpty()) continue;
//             numTags++;
//             avgDist +=
//                     tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
//         }
//         if (numTags == 0) return estStdDevs;
//         avgDist /= numTags;
//         // Decrease std devs if multiple targets are visible
//         if (numTags > 1) estStdDevs = kMultiTagStdDevs;
//         // Increase std devs based on (average) distance
//         if (numTags == 1 && avgDist > 4)
//             estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
//         else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

//         return estStdDevs;
//     }

//     // ----- Simulation

//     public void simulationPeriodic(Pose2d robotSimPose) {
//         visionSim.update(robotSimPose);
//     }

//     /** Reset pose history of the robot in the vision system simulation. */
//     public void resetSimPose(Pose2d pose) {
//         if (SwerveDriveTelemetry.isSimulation) visionSim.resetRobotPose(pose);
//     }

//     /** A Field2d for visualizing our robot and objects on the field. */
//     public Field2d getSimDebugField() {
//         if (!SwerveDriveTelemetry.isSimulation) return null;
//         return visionSim.getDebugField();
//     }
// }

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.awt.Desktop;
import java.net.URI;
import java.net.URISyntaxException;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;


public class Vision {
    private static final Vision instance = new Vision();

    private static Field2d field;

    private static PhotonCamera camera;

    private static AprilTagFieldLayout fieldLayout;
    private static PhotonPoseEstimator poseEstimator;

    private static Transform3d robotToCam;
    private static Translation3d robotToCamTranslation;
    private static Rotation3d robotToCamRotation;

    private static VisionSystemSim visionSim;
    private static SimCameraProperties cameraProp;
    private static PhotonCameraSim simCam;

    public static Pose2d robotPose = new Pose2d();

    private Vision() {
        field = new Field2d();

        camera = new PhotonCamera("Arducam_OV9281_USB_Camera"); // Rename this to whatever your camera is named.

        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
        robotToCamTranslation = new Translation3d(-.264, 0.0, -0.17);
        robotToCamRotation = new Rotation3d(0,0,6.28318531);
        robotToCam = new Transform3d(robotToCamTranslation, robotToCamRotation);

        poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("Vision");
            visionSim.addAprilTags(fieldLayout);

            cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
            cameraProp.setCalibError(0.25, 0.08);
            cameraProp.setFPS(30);
            cameraProp.setAvgLatencyMs(35);
            cameraProp.setLatencyStdDevMs(5);

            simCam = new PhotonCameraSim(camera, cameraProp);
            simCam.enableDrawWireframe(true);

            visionSim.addCamera(simCam, robotToCam);

            openSimCameraView();
        }
    }

    public static Vision getInstance() {
        return instance;
    }

    static double longDistancePoseEstimationCount = 0;

    public static Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> poseEst = filterPose(poseEstimator.update());

        if (poseEst.isPresent()) {
            field.getObject("Estimated pose").setPose(poseEst.get().estimatedPose.toPose2d());
        }

        return poseEst;
    }

    private static Optional<EstimatedRobotPose> filterPose(Optional<EstimatedRobotPose> pose) {
        if (pose.isPresent()) {
            double bestTargetAmbiguity = 1; // 1 is max ambiguity
            for (PhotonTrackedTarget target : pose.get().targetsUsed) {
                double ambiguity = target.getPoseAmbiguity();
                if (ambiguity != -1 && ambiguity < bestTargetAmbiguity) bestTargetAmbiguity = ambiguity;
            }
            if (bestTargetAmbiguity > 0.3) return Optional.empty();

            if (getDistanceFromPose(pose.get().estimatedPose.toPose2d()) > 1) {
                longDistancePoseEstimationCount++;
                if (longDistancePoseEstimationCount < 10) {
                    return Optional.empty();
                }
            } else {
                longDistancePoseEstimationCount = 0;
            }
            return pose;
        }
        return Optional.empty();
    }

    public static PhotonPipelineResult getLatestResult() {
        if (Robot.isReal()) {
            return camera.getLatestResult();
        }
        return simCam.getCamera().getLatestResult();
    }

    public static boolean hasTargets() {
        return getLatestResult().hasTargets();
    }

    public static double getDistanceFromPose(Pose2d pose) {
        return PhotonUtils.getDistanceToPose(robotPose, pose);
    }

    public static Pose2d getTagPose(int id) {
        Optional<Pose3d> tag = fieldLayout.getTagPose(id);
        if (tag.isPresent()) {
            return tag.get().toPose2d();
        }
        return null;
    }

    public static double getDistanceFromAprilTag(int id) {
        Optional<Pose3d> tag = fieldLayout.getTagPose(id);
        if (tag.isPresent()) {
            return getDistanceFromPose(tag.get().toPose2d());
        }
        return -1;
    }

    public static PhotonTrackedTarget getTargetFromId(int id) {
        PhotonTrackedTarget target = null;
        PhotonPipelineResult result = getLatestResult();
        if (result.hasTargets()) {
            for (PhotonTrackedTarget i : result.getTargets()) {
                if (i.getFiducialId() == id) {
                    target = i;
                }
            }
        }
        return target;
    }

    public static VisionSystemSim getVisionSim() {
        return visionSim;
    }

    private void openSimCameraView() {
        if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
            try {
                Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
            } catch (IOException | URISyntaxException e) {
                e.printStackTrace();
            }
        }
    }

    public static Field2d getVisionField() {
        return field;
    }

    public static void updateVisionField() {
        SmartDashboard.putData("Vision/field", field);

        List<PhotonTrackedTarget> targets = new ArrayList<>();
        if (hasTargets()) targets.addAll(getLatestResult().targets);

        List<Pose2d> poses = new ArrayList<>();
        for (PhotonTrackedTarget target : targets) {
            Pose2d targetPose = getTagPose(target.getFiducialId());
            poses.add(targetPose);
        }

        field.getObject("tracked targets").setPoses(poses);
        field.setRobotPose(robotPose);
    }
}
