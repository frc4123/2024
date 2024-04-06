package frc.robot;

import frc.robot.Constants.InputConstants;
import frc.robot.Constants.DrivingConstants;
import frc.robot.Constants.OperatorConstants;

import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ClosedShooter;
import frc.robot.subsystems.Skipper;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.ArmOpen;
// import frc.robot.subsystems.Vision;

import frc.robot.commands.auto.SweepAutoBlue;
import frc.robot.commands.auto.SweepAutoRed;
import frc.robot.commands.auto.FourNoteAuto;
import frc.robot.commands.auto.TaxiRightRed;
import frc.robot.commands.auto.TwoNoteLongBlue;
import frc.robot.commands.auto.TwoNoteLongRed;
import frc.robot.commands.auto.TaxiLeftRed;
import frc.robot.commands.auto.TaxiRightBlue;
import frc.robot.commands.auto.TaxiLeftBlue;

import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.intake.IntakeOut;
import frc.robot.commands.shooter.AutoShooter;
// import frc.robot.commands.shooter.ShootSpeaker;
// import frc.robot.commands.shooter.ShootAmp;
import frc.robot.commands.shooter.ClosedShootAmp;
import frc.robot.commands.shooter.ClosedShootSpeaker;
// import frc.robot.commands.shooter.AutoShooter;
import frc.robot.commands.skipper.SkipShooter;
import frc.robot.commands.skipper.AutoSkipShooter;
import frc.robot.commands.skipper.SkipAmp;
import frc.robot.commands.climber.ClimbDown;
import frc.robot.commands.climber.ClimbUp;
import frc.robot.commands.climber.RightClimbDown;
import frc.robot.commands.climber.RightClimbUp;
import frc.robot.commands.climber.LeftClimbDown;
import frc.robot.commands.climber.LeftClimbUp;
import frc.robot.commands.arm.ArmAutoSpeaker;
import frc.robot.commands.arm.ArmBrakeMode;
import frc.robot.commands.arm.ArmCoastMode;
import frc.robot.commands.arm.ArmIntake;
import frc.robot.commands.arm.ArmPlace;
import frc.robot.commands.arm.ArmSafe;
import frc.robot.commands.arm.ArmShoot;
// import frc.robot.commands.arm.ArmUp;
// import frc.robot.commands.arm.ArmDown;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;


public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));

  private final Intake m_intake = new Intake();
  private final ClosedShooter m_closedShooter = new ClosedShooter();
        
  private final Skipper m_skipper = new Skipper();
  private final Climber m_climber = new Climber();
  private final Arm m_arm = new Arm();
  // private final Vision m_vision = new Vision();

  private final CommandXboxController m_driverController1 = new CommandXboxController(InputConstants.kDriverControllerPort0);
  private final Joystick m_joystick = new Joystick(InputConstants.kDriverControllerPort1);
  private final CommandGenericHID m_buttonBoard = new CommandGenericHID(m_joystick.getPort());

  private final IntakeIn m_intakeIn = new IntakeIn(m_intake);
  private final IntakeOut m_intakeOut = new IntakeOut(m_intake);
  //private final ShootSpeaker m_shootSpeaker = new ShootSpeaker(m_shooter);
  //private final ShootAmp m_shootAmp = new ShootAmp(m_shooter);
  private final ClosedShootAmp m_closedAmp = new ClosedShootAmp(m_closedShooter);
  private final ClosedShootSpeaker m_closedSpeaker = new ClosedShootSpeaker(m_closedShooter);
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
  private final ArmIntake m_armIntake = new ArmIntake(m_arm);
  private final ArmPlace m_armPlace = new ArmPlace(m_arm);
  private final ArmShoot m_armShoot = new ArmShoot(m_arm);
  private final ArmSafe m_armSafe = new ArmSafe(m_arm);
  private final ArmAutoSpeaker m_armAutoSpeaker = new ArmAutoSpeaker(m_arm);
  // private final ArmUp m_ArmUp = new ArmUp(m_arm);
  // private final ArmDown m_ArmDown = new ArmDown(m_arm);
  //private final VisionAlign m_VisionAlign = new VisionAlign(m_vision);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  
  public RobotContainer() {

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(m_driverController1.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController1.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController1.getRightX() * 0.5);

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(m_driverController1.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController1.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController1.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);

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
    m_driverController1.y().whileTrue(m_armSafe); // sets arm to safe position while driving - diego was here
    m_driverController1.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));

    //m_buttonBoard.axisGreaterThan(0, 0.5).whileTrue(m_shootSpeaker);
    //m_buttonBoard.axisLessThan(0, -0.5).whileTrue(m_intakeOut);

    m_buttonBoard.button(1).whileTrue(m_intakeIn);
    m_buttonBoard.button(1).whileTrue(m_armIntake);

    m_buttonBoard.button(2).whileTrue(m_armShoot);
    m_buttonBoard.button(2).whileTrue(m_closedSpeaker);
    //m_buttonBoard.button(2).whileTrue(m_shootSpeaker);
    m_buttonBoard.button(2).whileTrue(new WaitCommand(0.6).andThen(m_skipShooter)); // 0.8

    //m_buttonBoard.button(3).whileTrue(m_shootAmp);
    m_buttonBoard.button(2).whileTrue(m_closedAmp);
    m_buttonBoard.button(3).whileTrue(new WaitCommand(.2).andThen(m_skipAmp));;
    m_buttonBoard.button(3).whileTrue(m_armPlace);

    m_buttonBoard.button(5).whileTrue(m_intakeOut);

    m_buttonBoard.button(6).whileTrue(m_climbUp);
    m_buttonBoard.button(7).whileTrue(m_climbDown);
    //m_buttonBoard.axisGreaterThan(0, 0.5).whileTrue(m_ArmUp);
    //m_buttonBoard.axisLessThan(0, 0.5).whileTrue(m_ArmDown);
    
  }

  //------------------------------------ Auto ------------------------------------//

  public void initializeAutoChooser(){
    m_autoChooser.setDefaultOption(
      "1 Note",new WaitCommand(0.1)
        .andThen(new ArmShoot(m_arm).withTimeout(3))
        //.alongWith(new ShootSpeaker(m_shooter).withTimeout(3))
        .alongWith(new AutoSkipShooter(m_skipper).withTimeout(3))
        .beforeStarting(new AutoShooter(m_closedShooter).withTimeout(1))
        );

    m_autoChooser.addOption(
      "Sweep Blue", new WaitCommand(0.1)
      .andThen(new SweepAutoBlue(drivebase).sweepAutoBlue())
      );

    m_autoChooser.addOption(
      "Sweep Red", new WaitCommand(0.1)
      .andThen(new SweepAutoRed(drivebase).sweepAutoRed())
      );

    // m_autoChooser.addOption(
    //   "Shoot then Sweep", new WaitCommand(0.1)
    //     .andThen(new ArmInitialize(m_arm).withTimeout(0.5))
    //     .andThen(new ArmShoot(m_arm).withTimeout(3))
    //     .alongWith(new AutoSkipShooter(m_skipper).withTimeout(3))
    //     .andThen(new ArmIntake(m_arm).withTimeout(0.5))
    //     //
    //     .andThen(new SweepAuto(m_swerveSubsystem).sweepAuto())
    //     .beforeStarting(new AutoShooter(m_shooter))
    // );
    m_autoChooser.addOption(
      "Taxi Red Right", new WaitCommand(0.1)
        .andThen(new AutoShooter(m_closedShooter).withTimeout(0.1))
      .andThen(new ArmShoot(m_arm).withTimeout(3))
        //.alongWith(new ShootSpeaker(m_shooter).withTimeout(3))
        .alongWith(new AutoSkipShooter(m_skipper).withTimeout(3))

      .beforeStarting(new WaitCommand(9)) 
      .andThen(new TaxiRightRed(drivebase).taxiRightRed())
    );

    m_autoChooser.addOption(
      "Taxi Red Left", new WaitCommand(0.1)
      .andThen(new AutoShooter(m_closedShooter).withTimeout(0.1))
      .andThen(new ArmShoot(m_arm).withTimeout(3))
      //.alongWith(new ShootSpeaker(m_shooter).withTimeout(3))
      .alongWith(new AutoSkipShooter(m_skipper).withTimeout(3))

      .beforeStarting(new WaitCommand(4)) 
      .andThen(new TaxiLeftRed(drivebase).taxiLeftRed())
    );

    m_autoChooser.addOption(
      "Taxi Blue Right", new WaitCommand(0.1)
      .andThen(new AutoShooter(m_closedShooter).withTimeout(0.1))
      .andThen(new ArmShoot(m_arm).withTimeout(3))
      //.alongWith(new ShootSpeaker(m_shooter).withTimeout(3))
      .alongWith(new AutoSkipShooter(m_skipper).withTimeout(3))

      .beforeStarting(new WaitCommand(4)) 
      .andThen(new TaxiLeftBlue(drivebase).taxiLeftBlue())
    );

    m_autoChooser.addOption(
      "Taxi Blue Left", new WaitCommand(0.1)
      .andThen(new AutoShooter(m_closedShooter).withTimeout(0.1))
      .andThen(new ArmShoot(m_arm).withTimeout(3))
      //.alongWith(new ShootSpeaker(m_shooter).withTimeout(3))
      .alongWith(new AutoSkipShooter(m_skipper).withTimeout(3))

      .beforeStarting(new WaitCommand(9)) 
      .andThen(new TaxiRightBlue(drivebase).taxiRightBlue())
    );


    // m_autoChooser.addOption(
    //   "Four Note Middle", new SequentialCommandGroup(
    //     new WaitCommand(0.001),
    //     //shoot first note
    //       new ParallelCommandGroup(new WaitCommand(2.75).andThen(new FourNoteAuto(m_swerveSubsystem).fourNote())),
    //       new SequentialCommandGroup(new AutoShooter(m_closedShooter).withTimeout(0.2)
    //       .andThen(new ArmShoot(m_arm).withTimeout(2.75))
    //       .alongWith(new AutoSkipShooter(m_skipper).withTimeout(2.75))
    //       //intake and shoot second note
    //       .andThen(new ArmIntake(m_arm).withTimeout(0.2)) // 0.2 (total seconds) 
    //       .andThen(new IntakeIn(m_intake).withTimeout(1.5)) // 1.7
    //       .andThen(new ArmShoot(m_arm).withTimeout(0.6)) //2.3
    //       .andThen(new SkipShooter(m_skipper).withTimeout(0.5)) // 2.8
    //       //intake and shoot third note
    //       .andThen(new ArmIntake(m_arm).withTimeout(0.3)) // 3.1
    //       .andThen(new IntakeIn(m_intake).withTimeout(0.7)) // 3.8
    //       .andThen(new ArmShoot(m_arm).withTimeout(1.23)) // 5.03
    //       .andThen(new SkipShooter(m_skipper).withTimeout(0.57)) // 5.6
    //       .andThen(new ArmIntake(m_arm).withTimeout(0.4)) // 6.0
    //       //intake and shoot fourth note
    //       .andThen(new IntakeIn(m_intake).withTimeout(1.1)) // 7.1
    //       .andThen(new ArmShoot(m_arm).withTimeout(1.2)) // 8.3
    //       .andThen(new SkipShooter(m_skipper).withTimeout(2.2))))// 10.5 // end of auto
    // );

        m_autoChooser.addOption(
      "Four Note Middle V3", new ParallelCommandGroup(
        new WaitCommand(0.001),
        //shoot first note
          new SequentialCommandGroup(new WaitCommand(2.7).andThen(new FourNoteAuto(drivebase).fourNote())),
          new SequentialCommandGroup(new AutoShooter(m_closedShooter).withTimeout(0.2)
          .andThen(new ArmShoot(m_arm).withTimeout(2.70))
          .alongWith(new AutoSkipShooter(m_skipper).withTimeout(2.70))
          //intake and shoot second note
          .andThen(new ArmIntake(m_arm).withTimeout(0.2)) // 0.2 (total seconds) 
          .andThen(new IntakeIn(m_intake).withTimeout(1.45)) // 1.55
          .andThen(new ArmShoot(m_arm).withTimeout(0.5)) //2.25
          .andThen(new SkipShooter(m_skipper).withTimeout(0.35)) // 2.7
          //intake and shoot third note
          .andThen(new ArmIntake(m_arm).withTimeout(0.6)) // 3.1
          .andThen(new IntakeIn(m_intake).withTimeout(0.7)) // 3.8
          .andThen(new ArmAutoSpeaker(m_arm).withTimeout(0.65)) // 4.5
          .andThen(new SkipShooter(m_skipper).withTimeout(.55)) // 5.6
          .andThen(new ArmIntake(m_arm).withTimeout(0.5)) // 6.0
          //intake and shoot fourth note
          .andThen(new IntakeIn(m_intake).withTimeout(1.6)) // 7.1
          .andThen(new ArmAutoSpeaker(m_arm).withTimeout(0.5)) // 8.3
          .andThen(new SkipShooter(m_skipper).withTimeout(2.2))))// 10.5 // end of auto
    );

//     m_autoChooser.addOption(
//     "Four Note Middle GPT", 
//     new SequentialCommandGroup(
//         new WaitCommand(0.001),
//         //shoot first note
//         new ParallelCommandGroup(
//             new WaitCommand(2.75).andThen(new FourNoteAuto(m_swerveSubsystem).fourNote()),
//             new ParallelCommandGroup(
//                 new AutoShooter(m_closedShooter).withTimeout(0.2),
//                 new ArmShoot(m_arm).withTimeout(2.75),
//                 new AutoSkipShooter(m_skipper).withTimeout(2.75),
//                 new ArmIntake(m_arm).withTimeout(0.2),
//                 new IntakeIn(m_intake).withTimeout(1.5),
//                 new ArmShoot(m_arm).withTimeout(0.6),
//                 new SkipShooter(m_skipper).withTimeout(0.5),
//                 new ArmIntake(m_arm).withTimeout(0.3),
//                 new IntakeIn(m_intake).withTimeout(0.7),
//                 new ArmShoot(m_arm).withTimeout(1.23),
//                 new SkipShooter(m_skipper).withTimeout(0.57),
//                 new ArmIntake(m_arm).withTimeout(0.4),
//                 new IntakeIn(m_intake).withTimeout(1.1),
//                 new ArmShoot(m_arm).withTimeout(1.2),
//                 new SkipShooter(m_skipper).withTimeout(2.2))
//             )
//     )
// );

    m_autoChooser.addOption("2 Long Red", new ParallelCommandGroup(
        new WaitCommand(0.001),
        new SequentialCommandGroup(new WaitCommand(1.1).andThen(new TwoNoteLongRed(drivebase).twoNoteLongRed())),
        new SequentialCommandGroup(new SequentialCommandGroup(new AutoShooter(m_closedShooter).withTimeout(0.2)
          .andThen(new ArmShoot(m_arm).withTimeout(1.1))
          .alongWith(new AutoSkipShooter(m_skipper).withTimeout(1.1)) // drivetrain starts after this
          // intake 2nd note
          .andThen(new ArmIntake(m_arm).withTimeout(0.5)) // 0.5
          .andThen(new IntakeIn(m_intake).withTimeout(4.0)) // 4.5
          // shoot 2nd note
          .andThen(new ArmAutoSpeaker(m_arm).withTimeout(3.0)) // 7
          .andThen(new SkipShooter(m_skipper).withTimeout(0.75)) // 7.75
          .andThen(new ArmIntake(m_arm).withTimeout(0.5)) // 8.2

    ))));

    m_autoChooser.addOption("2 Long Blue", new ParallelCommandGroup(
        new WaitCommand(0.001),
        new SequentialCommandGroup(new WaitCommand(2).andThen(new TwoNoteLongBlue(drivebase).twoNoteLongBlue())),
        new SequentialCommandGroup(new SequentialCommandGroup(new AutoShooter(m_closedShooter).withTimeout(0.2)
          .andThen(new ArmShoot(m_arm).withTimeout(2))
          .alongWith(new AutoSkipShooter(m_skipper).withTimeout(2)) // drivetrain starts after this
          // intake 2nd note
          .andThen(new ArmIntake(m_arm).withTimeout(0.5)) // 0.5
          .andThen(new IntakeIn(m_intake).withTimeout(4.0)) // 4.5
          // shoot 2nd note
          .andThen(new ArmAutoSpeaker(m_arm).withTimeout(3.2)) // 7.7
          .andThen(new SkipShooter(m_skipper).withTimeout(0.8)) // 8.2
          .andThen(new ArmIntake(m_arm).withTimeout(0.5)) // 9

    ))));

    // m_autoChooser.addOption(
    //   "FiveNote", new WaitCommand(0.1)
    //     .andThen(new ArmShoot(m_arm).withTimeout(2.5))
    //     .alongWith(new AutoSkipShooter(m_skipper).withTimeout(2.5))
    //     .andThen(new ArmIntake(m_arm).withTimeout(0.5))

    //     //creates two paralell commands during the auto, shooter, and path follower
    //     .beforeStarting(new AutoShooter(m_shooter))
    //     .beforeStarting(new WaitCommand(4)) 
    //       .andThen(new FiveNoteAuto(m_swerveSubsystem).fiveNote())

    // );

    // m_autoChooser.addOption(
    //   "Three Note Long", new WaitCommand(0.1)
    //     .andThen(new ArmShoot(m_arm).withTimeout(2.5))
    //     .alongWith(new AutoSkipShooter(m_skipper).withTimeout(3))
    //     .andThen(new ArmIntake(m_arm).withTimeout(0.5))

    //     //creates two paralell commands during the auto, shooter, and path follower
    //     .beforeStarting(new AutoShooter(m_shooter))
    //     .beforeStarting(new WaitCommand(4)) 
    //       .andThen(new ThreeNoteLong(m_swerveSubsystem).threeNoteLong())
    // );

    SmartDashboard.putData("Auto Selector", m_autoChooser);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}