package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Climber;

public class LeftClimbDown extends Command{
    Climber climb;

    public LeftClimbDown(Climber climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void execute(){
        climb.setLeftClimberVelo(0.8);
    }

    @Override
    public void end(boolean interrupted) {
        climb.setLeftClimberVelo(0.0); 
    }
}
