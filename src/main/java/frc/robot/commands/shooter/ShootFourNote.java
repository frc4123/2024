package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootFourNote extends Command{

    Shooter shooter;

    public ShootFourNote(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterVelo(0.7); 
    }
    
    @Override
    public boolean isFinished() {
        return DriverStation.isDisabled() || DriverStation.isAutonomousEnabled() == false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterVelo(0.0);
    }
}
