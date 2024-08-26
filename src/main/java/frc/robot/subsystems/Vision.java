package frc.robot.subsystems;

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
import frc.robot.Robot;
import java.awt.Desktop;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
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
import swervelib.SwerveDrive;
import swervelib.telemetry.Alert;
import swervelib.telemetry.Alert.AlertType;


/**
 * Example Vision class to aid in the pursuit of accurate odometry, using PhotonVision.
 */
public class Vision
{

  /**
   * Count of times that the odom thinks we're more than 10meters away from the april tag.
   */
  private double longDistangePoseEstimationCount = 0;

  /**
   * Camera class to manage the single camera setup.
   */
  public class Camera
  {
    /**
     * Transform of the camera rotation and translation relative to the center of the robot
     */
    private final Transform3d robotToCamTransform;

    /**
     * Latency alert to use when high latency is detected.
     */
    public final Alert latencyAlert;

    /**
     * Camera instance for comms.
     */
    public final PhotonCamera camera;
    
    /**
     * Simulated camera instance which only exists during simulations.
     */
    public PhotonCameraSim cameraSim;

    public Camera(String cameraName, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation) {
      latencyAlert = new Alert("'" + cameraName + "' Camera is experiencing high latency.", AlertType.WARNING);
  
      camera = new PhotonCamera(cameraName);
  
      // Define the transform of the camera relative to the robot
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);
  
      // Ensure you initialize the PhotonPoseEstimator with the camera instance
      poseEstimator = new PhotonPoseEstimator(
          fieldLayout, 
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
          camera,  // Pass the camera instance here
          robotToCamTransform
      );
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  
      if (Robot.isSimulation()) {
          SimCameraProperties cameraProp = new SimCameraProperties();
          // Camera properties initialization (same as before)
          cameraSim = new PhotonCameraSim(camera, cameraProp);
          cameraSim.enableDrawWireframe(true);
      }
  }

    /**
     * Pose estimator for camera.
     */
    public final PhotonPoseEstimator poseEstimator;

    /**
     * Construct a Photon Camera class with help.
     *
     * @param name                  Name of the PhotonVision camera found in the PV UI.
     * @param robotToCamRotation    {@link Rotation3d} of the camera.
     * @param robotToCamTranslation {@link Translation3d} relative to the center of the robot.
     */
  

    /**
     * Add camera to {@link VisionSystemSim} for simulated photon vision.
     *
     * @param systemSim {@link VisionSystemSim} to use.
     */
    public void addToVisionSim(VisionSystemSim systemSim)
    {
      if (Robot.isSimulation())
      {
        systemSim.addCamera(cameraSim, robotToCamTransform);
      }
    }
  }

  /**
   * April Tag Field Layout of the year.
   */
  private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

  /**
   * Photon Vision Simulation
   */
  public VisionSystemSim visionSim;

  /**
   * Current pose from the pose estimator using wheel odometry.
   */
  private Supplier<Pose2d> currentPose;

  /**
   * Field from {@link swervelib.SwerveDrive#field}
   */
  private Field2d field2d;

  /**
   * The single camera instance.
   */
  private final Camera camera;

  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference {@link SwerveDrive#getPose()}
   * @param field       Current field, should be {@link SwerveDrive#field}
   */
  public Vision(Supplier<Pose2d> currentPose, Field2d field)
  {
    this.currentPose = currentPose;
    this.field2d = field;

    camera = new Camera("Arducam_OV9281_USB_Camera", 
                        new Rotation3d(0, Units.degreesToRadians(23.511519), Units.degreesToRadians(360)),
                        new Translation3d(Units.inchesToMeters(-10.37),
                                          Units.inchesToMeters(6.678),
                                          Units.inchesToMeters(11.666192)));

    if (Robot.isSimulation())
    {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);
      camera.addToVisionSim(visionSim);
      openSimCameraViews();
    }
  }

  /**
   * Update the pose estimation inside of {@link SwerveDrive} with all of the given poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive)
  {
    for (EstimatedRobotPose i : getEstimatedGlobalPose())
    {
      swerveDrive.addVisionMeasurement(i.estimatedPose.toPose2d(), i.timestampSeconds);
    }
  }

  /**
   * Generates the estimated robot pose. Returns empty if:
   * <ul>
   *  <li> No Pose Estimates could be generated</li>
   * <li> The generated pose estimate was considered not accurate</li>
   * </ul>
   *
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used to create the estimate
   */
  public ArrayList<EstimatedRobotPose> getEstimatedGlobalPose()
  {
    ArrayList<EstimatedRobotPose> poses = new ArrayList<>();

    Optional<EstimatedRobotPose> poseEst = filterPose(camera.poseEstimator.update());

    if (poseEst.isPresent())
    {
      poses.add(poseEst.get());
      field2d.getObject("est pose").setPose(poseEst.get().estimatedPose.toPose2d());
    }

    return poses;
  }

  /**
   * Filter pose via the ambiguity and find best estimate throwing out distances more than 10m for a short amount of time.
   *
   * @param pose Estimated robot pose.
   * @return Could be empty if there isn't a good reading.
   */
  private Optional<EstimatedRobotPose> filterPose(Optional<EstimatedRobotPose> pose)
  {
    if (pose.isPresent())
    {
      double bestTargetAmbiguity = 1; // 1 is max ambiguity
      for (PhotonTrackedTarget target : pose.get().targetsUsed)
      {
        double ambiguity = target.getPoseAmbiguity();
        if (ambiguity != -1 && ambiguity < bestTargetAmbiguity)
        {
          bestTargetAmbiguity = ambiguity;
        }
      }
      //ambiguity too high don't use estimate
      if (bestTargetAmbiguity > 0.3)
      {
        return Optional.empty();
      }

      //est pose is very far from recorded robot pose
      if (PhotonUtils.getDistanceToPose(currentPose.get(), pose.get().estimatedPose.toPose2d()) > 1)
      {
        longDistangePoseEstimationCount++;

        //if it calculates that we're 10 meters away for more than 10 times in a row, it's probably right
        if (longDistangePoseEstimationCount < 10)
        {
          return Optional.empty();
        }
      } else
      {
        longDistangePoseEstimationCount = 0;
      }
      return pose;
    }
    return Optional.empty();
  }

  /**
   * Get the latest result from the camera.
   *
   * @return Photon result from sim or a real camera.
   */
  public PhotonPipelineResult getLatestResult()
  {
    return Robot.isReal() ? camera.camera.getLatestResult() : camera.cameraSim.getCamera().getLatestResult();
  }

  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id AprilTag ID
   * @return Distance
   */
  public double getDistanceFromAprilTag(int id)
  {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
  }

  /**
   * Get tracked target from the camera by AprilTag ID.
   *
   * @param id AprilTag ID
   * @return Tracked target.
   */
  public PhotonTrackedTarget getTargetFromId(int id)
  {
    PhotonTrackedTarget target = null;
    PhotonPipelineResult result = getLatestResult();
    if (result.hasTargets())
    {
      for (PhotonTrackedTarget i : result.getTargets())
      {
        if (i.getFiducialId() == id)
        {
          target = i;
        }
      }
    }
    return target;
  }

  /**
   * Vision simulation.
   *
   * @return Vision Simulation
   */
  public VisionSystemSim getVisionSim()
  {
    return visionSim;
  }

  /**
   * Open up the photon vision camera streams on the localhost, assumes running photon vision on localhost.
   */
  private void openSimCameraViews()
  {
    if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE))
    {
      try
      {
        Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
      } catch (IOException | URISyntaxException e)
      {
        e.printStackTrace();
      }
    }
  }

  /**
   * Update the field2d to include tracked targets.
   */
  public void updateVisionField()
  {
    List<PhotonTrackedTarget> targets = new ArrayList<>();

    if (getLatestResult().hasTargets())
    {
      targets.addAll(getLatestResult().getTargets());
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets)
    {
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent())
      {
        Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
        poses.add(targetPose);
      }
    }

    field2d.getObject("tracked targets").setPoses(poses);
  }
}
