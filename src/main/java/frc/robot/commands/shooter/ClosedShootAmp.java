package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClosedShooter;

public class ClosedShootAmp extends Command{

    ClosedShooter shooter;

    public ClosedShootAmp(ClosedShooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setShooterVelo(Constants.PIDTuning.Shooter_Target_Velo / 4); 
        // one fourth velo of speaker shooting
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.setShooterVelo(Constants.PIDTuning.Shooter_Stopped_Velo); 
    }
}
