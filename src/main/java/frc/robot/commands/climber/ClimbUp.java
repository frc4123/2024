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
        if (climb.getClimberPosition() < 30) { //INSERT CORRECT POSITION THRESHOLD + 0.1
            climb.setClimberVelo(-0.8);
            System.out.println("climber Up");
        }
    }

    @Override
    public void end(boolean interrupted) {
        climb.setClimberVelo(0.0); 
    }
    
}
