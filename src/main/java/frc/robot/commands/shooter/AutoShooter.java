package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AutoShooter extends Command{

    Shooter shooter;

    public AutoShooter(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setShooterVelo(0.9);
    }

    public void teleopInit() {
        shooter.setShooterVelo(0.0);
    }
}
