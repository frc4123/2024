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
import frc.robot.commands.arm.ArmBrakeMode;
import frc.robot.commands.arm.ArmCoastMode;
import frc.robot.commands.arm.ArmIntake;
import frc.robot.commands.arm.ArmPlace;
import frc.robot.commands.arm.ArmShoot;
import frc.robot.commands.swerve.Swerve;

import frc.robot.commands.arm.ArmOpenBackward;
import frc.robot.commands.arm.ArmOpenForward;
import frc.robot.subsystems.ArmOpenVelo;

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
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation;


public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Skipper m_skipper = new Skipper();
  private final Climber m_climber = new Climber();
  private final Arm m_arm = new Arm();
  private final ArmOpenVelo m_armOpenVelo = new ArmOpenVelo();

  private final CommandXboxController m_driverController1 = new CommandXboxController(InputConstants.kDriverControllerPort0);
  private final Joystick m_joystick = new Joystick(InputConstants.kDriverControllerPort1);
  private final CommandGenericHID m_buttonBoard = new CommandGenericHID(m_joystick.getPort());

  private final IntakeIn m_intakeIn = new IntakeIn(m_intake);
  private final Shoot m_shoot = new Shoot(m_shooter);
  private final Skip m_skip = new Skip(m_skipper);
  private final ClimbUp m_climbUp = new ClimbUp(m_climber);
  private final ClimbDown m_climbDown = new ClimbDown(m_climber);
  private final ArmBrakeMode m_armBrakeMode = new ArmBrakeMode(m_arm);
  private final ArmCoastMode m_armCoastMode = new ArmCoastMode(m_arm);
  Command m_armBrakeModeWrapped = m_armBrakeMode.ignoringDisable(true); // creates wrapped command for .ignoringDisable()
  Command m_armCoastModeWrapped = m_armCoastMode.ignoringDisable(true); // creates wrapped command for .ignoringDisable()
  private final ArmIntake m_ArmIntake = new ArmIntake(m_arm);
  private final ArmPlace m_ArmPlace = new ArmPlace(m_arm);
  private final ArmShoot m_ArmShoot = new ArmShoot(m_arm);

  private final ArmOpenBackward m_ArmOpenBackward = new ArmOpenBackward(m_armOpenVelo);
  private final ArmOpenForward m_ArmOpenForward = new ArmOpenForward(m_armOpenVelo);
  
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new Swerve(
                swerveSubsystem,
                () -> -m_driverController1.getLeftY() * DrivingConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
                () -> -m_driverController1.getLeftX() * DrivingConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
                () -> -m_driverController1.getRawAxis(4) *  DrivingConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond
                ));
    configureBindings();
  }

  private void configureBindings() {
    if (DriverStation.isEnabled() == false); {
      m_buttonBoard.button(1).whileTrue(m_armBrakeModeWrapped);
      m_buttonBoard.button(2).whileTrue(m_armCoastModeWrapped);
    }

    m_buttonBoard.button(1).whileTrue(m_intakeIn);
    m_buttonBoard.button(2).whileTrue(m_shoot);
    m_buttonBoard.button(2).whileTrue(new WaitCommand(0.8).andThen(m_skip));
    m_buttonBoard.button(3).whileTrue(m_ArmOpenForward);
    m_buttonBoard.button(4).whileTrue(m_ArmOpenBackward);
    m_buttonBoard.button(11).whileTrue(m_climbUp);
    m_buttonBoard.button(11).whileTrue(m_climbDown);
    m_buttonBoard.button(11).whileTrue(m_ArmIntake);
    m_buttonBoard.button(11).whileTrue(m_ArmShoot);
    m_buttonBoard.button(11).whileTrue(m_ArmPlace);
    //m_buttonBoard.axisGreaterThan(0, 0.5).whileTrue(m_climbUp);
    //m_buttonBoard.axisLessThan(0, 0.5).whileTrue(m_climbDown);
    
  }






  //------------------------------------ Auto ------------------------------------//






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
