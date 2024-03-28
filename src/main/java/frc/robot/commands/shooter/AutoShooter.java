package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.DriverStation;
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
        if (DriverStation.isAutonomous()) {
            shooter.setShooterVelo(-0.7);
        }

    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterVelo(0.0);
    }
}
