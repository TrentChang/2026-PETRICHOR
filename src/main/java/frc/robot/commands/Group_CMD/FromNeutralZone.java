package frc.robot.commands.Group_CMD;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constant.flyWheelConstant;
import frc.robot.commands.Single_CMD.SmartTransport;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;

public class FromNeutralZone extends SequentialCommandGroup{
    public FromNeutralZone(Hood hood, Flywheel flywheel, Conveyor conveyor, Intake intake) {
        addCommands(new InstantCommand(hood::fromNeutralZone, hood));
        addCommands(flywheel.spinUpCommand(flyWheelConstant.speedLimit));
        addCommands(flywheel.backupSpinupCommand(flyWheelConstant.fromNeutralZoneRPM).alongWith(new InstantCommand(conveyor::converyorTransport, conveyor)));
        addCommands(new SmartTransport(intake));
    }
}
