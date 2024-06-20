package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase
{
    public static ArmSubsystem instance;

    public static ArmSubsystem getInstance()
        {   if(instance == null) instance = new ArmSubsystem();
            return instance;}

    public enum ArmDirection{
        OPEN,CLOSE,HOLD,FREE
    }
    public enum ArmPosition
    {
        HIGH(Constants.ArmConstants.ARM_ENCODER_POSITION_HIGH),MID(Constants.ArmConstants.ARM_ENCODER_POSITION_MIDE);

        double position;

        ArmPosition(double position) {this.position = position;}

        public double getValue() {return position;}
    }

    DigitalInput ground,fold;
    CANSparkMax liftMotor;

    private RelativeEncoder encoder;
    private ArmFeedforward ff;

    ArmDirection direction;

    private ArmSubsystem()
    {
        ff = new ArmFeedforward(0, Constants.ArmConstants.FF_KG , 0);
        liftMotor = new CANSparkMax(Constants.ArmConstants.LIFT_MOTOR_PORT, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        ground = new DigitalInput(Constants.ArmConstants.GROUND_SWICH_PORT);
        fold = new DigitalInput(Constants.ArmConstants.FOLD_SWICH_PORT);
        encoder = liftMotor.getEncoder();
    }

    double resistGravity(){
        return ff.calculate(encoder.getPosition(), 0);
    }

    void setSpeed(double speed)
    {
        liftMotor.set(speed);
    }
    public ArmDirection getDirection() {return direction;}

    public Command open(double speed)
        {return Commands.startEnd(() -> setSpeed(speed + resistGravity()),() -> setSpeed(0),instance).until(() -> !ground.get());}

    public Command close(double speed)
        {return Commands.startEnd(() -> setSpeed(speed + resistGravity()),() -> setSpeed(0),instance).until(() -> !fold.get());}

    public Command goToPosition(double speed,ArmPosition position)
    {
        direction = encoder.getPosition() < position.getValue() ? ArmDirection.OPEN : ArmDirection.CLOSE;
        return Commands.startEnd(() -> setSpeed(speed + resistGravity()),() -> setSpeed(resistGravity()),instance)
                .until(() -> (direction==ArmDirection.CLOSE ? encoder.getPosition()>position.getValue() : encoder.getPosition()<position.getValue()));
    }
}