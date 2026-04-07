package frc.robot.commands.Single_CMD;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class SmartTransport extends Command{
    private final Intake intake;

    public SmartTransport(Intake intake) {
        this.intake = intake;

        addRequirements(this.intake);
    }

    @Override
    public void execute() {
        intake.intakeInhale();
        intake.intakeSystole();
    }
}   
