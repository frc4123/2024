package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootAmp extends Command{

    Shooter shooter;

    public ShootAmp(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setShooterVelo(0.1); 
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.setShooterVelo(0.0); 
    }
}
