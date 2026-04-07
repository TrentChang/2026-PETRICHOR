package frc.robot.commands.Auto_CMD;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Single_CMD.SmartTransport;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;

public class AutoShoot extends SequentialCommandGroup{
    public AutoShoot(Flywheel flywheel, Conveyor conveyor, Intake intake, Hood hood, CommandSwerveDrivetrain drivetrain) {
        addCommands(hood.hoodSetPos(drivetrain::getDistanceToTarget));
        addCommands(flywheel.spinUpCommand(1800));
        addCommands(flywheel.dashboardSpinUpCommand(drivetrain.getDistanceToTarget())
                    .alongWith(new SmartTransport(intake))
                    .alongWith(new InstantCommand(conveyor::converyorTransport, conveyor)));
    }
}
