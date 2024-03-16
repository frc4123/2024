package frc.robot;

import frc.robot.Constants.InputConstants;
import frc.robot.Constants.DrivingConstants;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Skipper;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Vision;

import frc.robot.subsystems.SwerveSubsystem;

import frc.robot.commands.auto.DriveToNote;
import frc.robot.commands.auto.DriveToSpeaker;
import frc.robot.commands.auto.Taxi;
import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.shooter.ShootSpeaker;
import frc.robot.commands.shooter.ShootAmp;
import frc.robot.commands.skipper.SkipShooter;
import frc.robot.commands.skipper.AutoSkipShooter;
import frc.robot.commands.skipper.SkipAmp;
import frc.robot.commands.climber.ClimbDown;
import frc.robot.commands.climber.ClimbUp;
import frc.robot.commands.climber.RightClimbDown;
import frc.robot.commands.climber.RightClimbUp;
import frc.robot.commands.climber.LeftClimbDown;
import frc.robot.commands.climber.LeftClimbUp;
import frc.robot.commands.arm.ArmBrakeMode;
import frc.robot.commands.arm.ArmCoastMode;
import frc.robot.commands.arm.ArmIntake;
import frc.robot.commands.arm.ArmPlace;
import frc.robot.commands.arm.ArmSafe;
import frc.robot.commands.arm.ArmShoot;
import frc.robot.commands.arm.ArmInitialize;
import frc.robot.commands.swerve.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;


public class RobotContainer {
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Skipper m_skipper = new Skipper();
  private final Climber m_climber = new Climber();
  private final Arm m_arm = new Arm();
  // private final Vision m_vision = new Vision();
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

  private final CommandXboxController m_driverController1 = new CommandXboxController(InputConstants.kDriverControllerPort0);
  private final Joystick m_joystick = new Joystick(InputConstants.kDriverControllerPort1);
  private final CommandGenericHID m_buttonBoard = new CommandGenericHID(m_joystick.getPort());

  private final IntakeIn m_intakeIn = new IntakeIn(m_intake);
  private final ShootSpeaker m_shootSpeaker = new ShootSpeaker(m_shooter);
  private final ShootAmp m_shootAmp = new ShootAmp(m_shooter);
  private final SkipShooter m_skipShooter = new SkipShooter(m_skipper);
  private final SkipAmp m_skipAmp = new SkipAmp(m_skipper);
  private final ClimbUp m_climbUp = new ClimbUp(m_climber);
  private final ClimbDown m_climbDown = new ClimbDown(m_climber);
  private final RightClimbUp m_RightClimbUp = new RightClimbUp(m_climber);
  private final RightClimbDown m_RightClimbDown = new RightClimbDown(m_climber);
  private final LeftClimbUp m_LeftClimbUp = new LeftClimbUp(m_climber);
  private final LeftClimbDown m_LeftClimbDown = new LeftClimbDown(m_climber);
  Command m_rightClimbUpWrapped = m_RightClimbUp.ignoringDisable(true);
  Command m_rightClimbDownWrapped = m_RightClimbDown.ignoringDisable(true);
  Command m_leftClimbUpWrapped = m_LeftClimbUp.ignoringDisable(true);
  Command m_leftClimbDownWrapped = m_LeftClimbDown.ignoringDisable(true);
  private final ArmBrakeMode m_armBrakeMode = new ArmBrakeMode(m_arm);
  private final ArmCoastMode m_armCoastMode = new ArmCoastMode(m_arm);
  Command m_armBrakeModeWrapped = m_armBrakeMode.ignoringDisable(true); // creates wrapped command for .ignoringDisable()
  Command m_armCoastModeWrapped = m_armCoastMode.ignoringDisable(true); // creates wrapped command for .ignoringDisable()
  private final ArmIntake m_ArmIntake = new ArmIntake(m_arm);
  private final ArmPlace m_ArmPlace = new ArmPlace(m_arm);
  private final ArmShoot m_ArmShoot = new ArmShoot(m_arm);
  private final ArmSafe m_ArmSafe = new ArmSafe(m_arm);
  //private final VisionAlign m_VisionAlign = new VisionAlign(m_vision);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  
  public RobotContainer() {
    m_swerveSubsystem.setDefaultCommand(new Swerve(
                m_swerveSubsystem,
                () -> -m_driverController1.getLeftY() * DrivingConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
                () -> -m_driverController1.getLeftX() * DrivingConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
                () -> -m_driverController1.getRawAxis(4) *  DrivingConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond
                ));
    configureBindings();
    initializeAutoChooser();
  }

  private void configureBindings() {
    // disabled commands
    if (DriverStation.isEnabled() == false) {
      m_driverController1.povUp().whileTrue(m_armBrakeModeWrapped);
      m_driverController1.povDown().whileTrue(m_armCoastModeWrapped);

      //m_driverController2.a().whileTrue(m_rightClimbDownWrapped);
      //m_driverController2.b().whileTrue(m_rightClimbUpWrapped);
      //m_driverController2.y().whileTrue(m_leftClimbUpWrapped);
      //m_driverController2.x().whileTrue(m_leftClimbDownWrapped);
    }
    // enabled commands
    m_driverController1.y().whileTrue(m_ArmSafe); // sets arm to safe position while driving - diego was here
    m_buttonBoard.button(1).whileTrue(m_intakeIn);
    m_buttonBoard.button(2).whileTrue(m_shootAmp);
    m_buttonBoard.button(2).whileTrue(new WaitCommand(.2).andThen(m_skipAmp));;
    m_buttonBoard.button(3).whileTrue(m_ArmIntake);
    m_buttonBoard.button(4).whileTrue(m_ArmShoot);
    m_buttonBoard.button(4).whileTrue(m_shootSpeaker);
    m_buttonBoard.button(4).whileTrue(new WaitCommand(1.25).andThen(m_skipShooter)); // 0.8
    m_buttonBoard.button(5).whileTrue(m_ArmPlace);
    m_buttonBoard.button(6).whileTrue(m_climbUp);
    m_buttonBoard.button(7).whileTrue(m_climbDown);
    //m_buttonBoard.axisGreaterThan(0, 0.5).whileTrue(m_climbUp);
    //m_buttonBoard.axisLessThan(0, 0.5).whileTrue(m_climbDown);
    
  }

  //------------------------------------ Auto ------------------------------------//

  public void initializeAutoChooser(){
    m_autoChooser.setDefaultOption(
      "1 Note",new WaitCommand(0.1)
        .andThen(new ArmInitialize(m_arm).withTimeout(0.5))
        .andThen(new ArmShoot(m_arm).withTimeout(3))
        .alongWith(new ShootSpeaker(m_shooter).withTimeout(3))
        .alongWith(new AutoSkipShooter(m_skipper).withTimeout(3))
        .andThen(new ArmIntake(m_arm).withTimeout(0.5))
        );

    m_autoChooser.addOption(
      "2 Note",new WaitCommand(0.1)
        .andThen(new ArmInitialize(m_arm).withTimeout(0.5))
        .andThen(new ArmShoot(m_arm).withTimeout(3))
        .alongWith(new ShootSpeaker(m_shooter).withTimeout(3))
        .alongWith(new AutoSkipShooter(m_skipper).withTimeout(3))
        .andThen(new ArmIntake(m_arm).withTimeout(0.5))
        //
        .alongWith(new IntakeIn(m_intake).withTimeout(2.25))
        .alongWith(new DriveToNote(m_swerveSubsystem).withTimeout(3))
        .andThen(new DriveToSpeaker(m_swerveSubsystem).withTimeout(3))
        .andThen(new ArmShoot(m_arm).withTimeout(3))
        .alongWith(new ShootSpeaker(m_shooter).withTimeout(3))
        .alongWith(new AutoSkipShooter(m_skipper).withTimeout(3))
        .andThen(new ArmIntake(m_arm).withTimeout(0.5))
          ); 
  

    SmartDashboard.putData("Auto Selector", m_autoChooser);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
