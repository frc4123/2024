package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Climber;

public class ClimbUp extends Command{
    Climber climb;

    public ClimbUp(Climber climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void execute(){
        climb.setClimberVelo(-0.8);
    }

    @Override
    public void end(boolean interrupted) {
        climb.setClimberVelo(0.0); 
    }

    public void periodic () {
        if (climb.isUnsafeVelocityUp()) {
            end(true);
            System.out.println("ClimbUp Stopped");
        }
    }
}
