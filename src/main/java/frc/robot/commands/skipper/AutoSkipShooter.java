package frc.robot.commands.skipper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer; // Import Timer for delay

import frc.robot.subsystems.Skipper;

public class AutoSkipShooter extends Command {
    Skipper skipper;
    Timer timer = new Timer(); // Create a Timer object

    public AutoSkipShooter(Skipper skipper) {
        this.skipper = skipper;
        addRequirements(skipper);
    }

    @Override
    public void initialize() {
        timer.start(); // Start the timer
    }

    @Override
    public void execute() {
        if (timer.get() >= 1.8) { // Check if 1.8 second has passed
            skipper.setSkipperVelo(0.8);
        }
    }

    @Override
    public void end(boolean interrupted) {
        skipper.setSkipperVelo(0.0);
    }
}