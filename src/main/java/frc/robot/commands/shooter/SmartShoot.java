// package frc.robot.commands.shooter;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Vision;

// public class SmartShoot extends Command{

//     Shooter shooter;
//     Vision vision;

//     public SmartShoot(Shooter shooter, Vision vision) {
//         this.shooter = shooter;
//         this.vision = vision;

//         addRequirements(vision);
//         addRequirements(shooter);
//     }

//     @Override
//     public void execute() {
//         shooter.setShooterVelo(-0.7); 
//     }
    
//     @Override
//     public void end(boolean interrupted) {
//         shooter.setShooterVelo(0.0); 
//     }



//     public void periodic(){
//         if (vision.isNearShooter()) {
//             execute();
//         } else if (vision.hasTarget()){
//             new WaitCommand(1)
//             .andThen( new InstantCommand(() -> end(true)));
//         }
//     }
// }
