package frc.robot.commands.Single_CMD;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class SmartIntake extends Command{
    private final Intake intake;
    private final Conveyor conveyor;

    public SmartIntake(Intake intake, Conveyor conveyor) {
        this.intake = intake;
        this.conveyor = conveyor;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.intake, this.conveyor);
    }

    @Override
    public void execute() {
        conveyor.conveyorIntake();
        intake.intakeInhale();
        intake.intakeExtend();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
