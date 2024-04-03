package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClosedShooter;

public class AutoShooter extends Command{

    ClosedShooter shooter;

    public AutoShooter(ClosedShooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if (DriverStation.isAutonomousEnabled()) {
            shooter.setShooterVelo(Constants.PIDTuning.Shooter_Target_Velo); 
        }
        // one fourth velo of speaker shooting
    }

    @Override 
    public void end(boolean interrupted) {}
    
    // @Override
    // public void end(boolean interrupted) {
    //         new Thread(() -> {
    //             try {
    //                 Thread.sleep(100); //15000
    //                 if (DriverStation.isTeleopEnabled()){
    //                 shooter.setShooterVelo(Constants.PIDTuning.Shooter_Stopped_Velo); 
    //                 }
    //             } catch (Exception e) {}
    //         }).start();
    // }
}
