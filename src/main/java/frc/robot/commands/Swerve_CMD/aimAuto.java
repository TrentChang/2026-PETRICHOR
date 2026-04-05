package frc.robot.commands.Swerve_CMD;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;

public class aimAuto extends SequentialCommandGroup {
    public aimAuto(CommandSwerveDrivetrain drivetrain, CommandXboxController xboxController, Hood hood) {
        addCommands(drivetrain.autoAlignCommand(xboxController).alongWith(hood.hoodSetPos(drivetrain::getDistanceToTarget)));
    }
}
