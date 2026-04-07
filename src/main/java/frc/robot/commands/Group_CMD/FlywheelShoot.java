package frc.robot.commands.Group_CMD;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Single_CMD.SmartTransport;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;

public class FlywheelShoot extends SequentialCommandGroup{
    public FlywheelShoot(Flywheel flywheel, CommandSwerveDrivetrain drivetrain, Conveyor conveyor, Intake intake) {
        addCommands(flywheel.dashboardSpinUpCommand(drivetrain.getDistanceToTarget())
            .alongWith(new SmartTransport(intake))
            .alongWith(new InstantCommand(conveyor::converyorTransport, conveyor)));
    }
}

