package frc.robot;

import frc.robot.Constants.InputConstants;
import frc.robot.Constants.DrivingConstants;
import frc.robot.Constants.AutoConstants;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Skipper;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveSubsystem;


import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.skipper.Skip;
import frc.robot.commands.climber.ClimbDown;
import frc.robot.commands.climber.ClimbUp;
import frc.robot.commands.arm.ArmIntake;
import frc.robot.commands.arm.ArmPlace;
import frc.robot.commands.arm.ArmShoot;
import frc.robot.commands.arm.ArmStart;
import frc.robot.commands.swerve.Swerve;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;


import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj.Joystick; // joystick input button -> joystick.getRawButton((button#) axis -> m_joystick.getX() ;


public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Skipper m_skipper = new Skipper();
  private final Climber m_climber = new Climber();
  private final Arm m_arm = new Arm();

  private final CommandXboxController m_driverController = new CommandXboxController(InputConstants.kDriverControllerPort);

  private final IntakeIn m_intakeIn = new IntakeIn(m_intake);
  private final Shoot m_shoot = new Shoot(m_shooter);
  private final Skip m_skip = new Skip(m_skipper);
  private final ClimbUp m_climbUp = new ClimbUp(m_climber);
  private final ClimbDown m_climbDown = new ClimbDown(m_climber);
  private final ArmIntake m_ArmIntake = new ArmIntake(m_arm);
  private final ArmPlace m_ArmPlace = new ArmPlace(m_arm);
  private final ArmShoot m_ArmShoot = new ArmShoot(m_arm);
  private final ArmStart m_ArmStart = new ArmStart(m_arm);
  //private final Joystick m_joystick = new Joystick(InputConstants.kDriverControllerPort);

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
                () -> -m_driverController.getRawAxis(4) *  DrivingConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond
                ));
    configureBindings();
  }
    /* public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new Swerve(
                swerveSubsystem,
                () -> -m_joystick.getRawAxis(1) * DrivingConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
                () -> -m_joystick.getRawAxis(0) * DrivingConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
                () -> -m_joystick.getRawAxis(2) *  DrivingConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond
                ));
    configureBindings();
  } */

  private void configureBindings() {
    
    m_driverController.a().whileTrue(m_intakeIn);
    m_driverController.y().whileTrue(m_shoot);
    m_driverController.y().whileTrue(new WaitCommand(0.5).andThen(m_skip));
    m_driverController.povUp().whileTrue(m_climbUp);
    m_driverController.povDown().whileTrue(m_climbDown);

  
  
    //CommandScheduler.getInstance().schedule(new Shooter(m_shoot), 1);
    //m_driverController.y().whileTrue(m_skip);
    
    // driverController2.rightBumper().onTrue(new WaitCommand(0.1).andThen(m_toggleSet).withTimeout(0.5));
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
