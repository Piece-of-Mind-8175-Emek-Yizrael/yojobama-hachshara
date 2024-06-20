package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

import java.util.function.BooleanSupplier;

public class IntakeSubsystem extends SubsystemBase
{
    public static IntakeSubsystem instance;

    public static IntakeSubsystem getInstance()
        {   if(instance == null) instance = new IntakeSubsystem();
            return instance;}

    CANSparkMax intakeMotor;
    boolean isIn = false;

    private IntakeSubsystem() {intakeMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_PORT, CANSparkLowLevel.MotorType.kBrushless);}

    void spin(double speed) {intakeMotor.set(speed);}

    public Command intakeUntilIn(double speed)
        {return Commands.startEnd(() -> spin(speed),() -> spin(0),instance)
                .until(() -> (intakeMotor.getOutputCurrent() > Constants.IntakeConstants.INTAKE_CURRENT_LIMIT)).finallyDo(() -> isIn=true);}

    public Command outtake(double speed)
        {return Commands.startEnd(() -> spin(speed),() -> spin(0),instance).finallyDo(() -> isIn=false);}

    public Command outtakeTime(double seconds,double speed)
        {return Commands.startEnd(() -> spin(speed),() -> spin(0),instance)
                .deadlineWith(new WaitCommand(seconds))
                    .finallyDo(() -> isIn=false);}

    public boolean getIsIn() {return isIn;}
}
