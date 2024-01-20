package frc.robot;

import frc.robot.Constants.InputConstants;
import frc.robot.Constants.DrivingConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.Swerve;

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj.Joystick; // joystick input button -> joystick.getRawButton((button#) axis -> m_joystick.getX() ;


public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final CommandXboxController m_driverController = new CommandXboxController(InputConstants.kDriverControllerPort);
  //private final Joystick m_joystick = new Joystick(InputConstants.kDriverControllerPort2);
  /* joystick.getRawAxis(0);  X-axis
  joystick.getRawAxis(1);  Y-axis
  joystick.getRawAxis(2);  wrist (rudder) axis
  joystick.getRawAxis(3);  Slider axis  
  joystick.getRawButton(#); Buttons 1-12 */



  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new Swerve(
                swerveSubsystem,
                () -> -m_driverController.getLeftY() * DrivingConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
                () -> -m_driverController.getLeftX() * DrivingConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
                () -> m_driverController.getRawAxis(4) *  DrivingConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond
                ));
    configureBindings();
  }

  private void configureBindings() {
  }






  // Auto

  public Command getAutonomousCommand() {
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DrivingConstants.kDriveKinematics);
            
    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
              new Translation2d(1, 0),
              new Translation2d(1, -1)),
      new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
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
          swerveSubsystem::getPose,
          DrivingConstants.kDriveKinematics,
          xController,
          yController,
          thetaController,
          swerveSubsystem::setModuleStates,
          swerveSubsystem);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
          new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
          swerveControllerCommand,
          new InstantCommand(() -> swerveSubsystem.stopModules()));   

  }
}
