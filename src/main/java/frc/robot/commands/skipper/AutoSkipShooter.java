package frc.robot.commands.skipper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.Skipper;

public class AutoSkipShooter extends Command{
    Skipper skipper;
    Timer timer = new Timer();

    public AutoSkipShooter(Skipper skipper) {
        this.skipper = skipper;
        addRequirements(skipper);
    }

    @Override
    public void execute() {
        timer.start();

        if (timer.get() >= 1.8) {
            skipper.setSkipperVelo(0.8);
        }
    }

    @Override
    public void end(boolean interrupted) {
        skipper.setSkipperVelo(0);
    }
}
