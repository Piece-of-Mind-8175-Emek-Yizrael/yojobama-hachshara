package pom.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance;
    CANSparkMax intakeMotor;
    boolean isIn = false;

    private IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_PORT, CANSparkLowLevel.MotorType.kBrushless);
    }

    public static IntakeSubsystem getInstance() {
        if (instance == null) instance = new IntakeSubsystem();
        return instance;
    }

    public Command intakeUntilIn(double speed) {
        return Commands.startEnd(() -> spin(speed), () -> spin(0), instance)
                .until(() -> (intakeMotor.getOutputCurrent() > Constants.IntakeConstants.INTAKE_CURRENT_LIMIT)).finallyDo(() -> isIn = true);
    }


    void spin(double speed) {
        intakeMotor.set(speed);
    }

    public Command outtake(double speed) {
        return Commands.startEnd(() -> spin(speed), () -> spin(0), instance).finallyDo(() -> isIn = false);
    }

    public Command outtakeTime(double seconds, double speed) {
        return Commands.startEnd(() -> spin(speed), () -> spin(0), instance)
                .deadlineWith(new WaitCommand(seconds))
                .finallyDo(() -> isIn = false);
    }

    public boolean getIsIn() {
        return isIn;
    }

}

