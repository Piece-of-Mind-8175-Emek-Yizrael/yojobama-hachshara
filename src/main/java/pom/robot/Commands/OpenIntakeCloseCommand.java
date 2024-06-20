package pom.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

public class OpenIntakeCloseCommand extends Command {
    IntakeSubsystem intakeSubsystem;
    ArmSubsystem armSubsystem;

    public OpenIntakeCloseCommand(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void initialize() {
        armSubsystem.open(Constants.ArmConstants.ARM_POWER)
                .andThen(intakeSubsystem.intakeUntilIn(Constants.IntakeConstants.INTAKE_SPIN_POWER_IN)
                        .andThen(armSubsystem.close(-Constants.ArmConstants.ARM_POWER)));
    }
}
