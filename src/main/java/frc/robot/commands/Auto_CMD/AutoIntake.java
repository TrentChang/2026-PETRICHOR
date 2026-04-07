package frc.robot.commands.Auto_CMD;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Single_CMD.SmartIntake;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class AutoIntake extends SequentialCommandGroup{
    public AutoIntake(Intake intake, Conveyor conveyor) {
        addCommands(new SmartIntake(intake, conveyor));
    }
}
