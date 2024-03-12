package frc.robot.commands.auto;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivingConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;



public class DriveToNote extends Command{
    SwerveSubsystem swerve;

    public DriveToNote(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    public Command followAutoTrajectory() {
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DrivingConstants.kDriveKinematics);
            
    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
              new Translation2d(0, 2)),
      new Pose2d(0, 2, Rotation2d.fromDegrees(0)),
      trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
          trajectory,
          swerve::getPose,
          DrivingConstants.kDriveKinematics,
          xController,
          yController,
          thetaController,
          swerve::setModuleStates,
          swerve);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
          swerveControllerCommand,
          new InstantCommand(() -> swerve.stopModules()));   

  }
    
}
