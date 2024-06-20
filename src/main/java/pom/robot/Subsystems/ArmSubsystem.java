package pom.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    private static ArmSubsystem instance;
    DigitalInput ground, fold;
    CANSparkMax liftMotor;
    ArmDirection direction;
    private RelativeEncoder encoder;
    private ArmFeedforward ff;

    private ArmSubsystem() {
        ff = new ArmFeedforward(0, Constants.ArmConstants.FF_KG, 0);
        liftMotor = new CANSparkMax(Constants.ArmConstants.LIFT_MOTOR_PORT, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        ground = new DigitalInput(Constants.ArmConstants.GROUND_SWICH_PORT);
        fold = new DigitalInput(Constants.ArmConstants.FOLD_SWICH_PORT);
        encoder = liftMotor.getEncoder();
    }

    public static ArmSubsystem getInstance() {
        if (instance == null) instance = new ArmSubsystem();
        return instance;
    }

    public ArmDirection getDirection() {
        return direction;
    }

    public Command open(double speed) {
        return Commands.startEnd(() -> setSpeed(speed + resistGravity()), () -> setSpeed(0), instance).until(() -> !ground.get());
    }

    void setSpeed(double speed) {
        liftMotor.set(speed);
    }

    double resistGravity() {
        return ff.calculate(encoder.getPosition(), 0);
    }

    public Command close(double speed) {
        return Commands.startEnd(() -> setSpeed(speed + resistGravity()), () -> setSpeed(0), instance).until(() -> !fold.get());
    }

    public Command goToPosition(double speed, ArmPosition position) {
        direction = encoder.getPosition() < position.getValue() ? ArmDirection.OPEN : ArmDirection.CLOSE;
        return Commands.startEnd(() -> setSpeed(speed + resistGravity()), () -> setSpeed(resistGravity()), instance)
                .until(() -> (direction == ArmDirection.CLOSE ? encoder.getPosition() > position.getValue() : encoder.getPosition() < position.getValue()));
    }

    public enum ArmDirection {
        OPEN, CLOSE, HOLD, FREE
    }

    public enum ArmPosition {
        HIGH(Constants.ArmConstants.ARM_ENCODER_POSITION_HIGH), MID(Constants.ArmConstants.ARM_ENCODER_POSITION_MIDE);

        double position;

        ArmPosition(double position) {
            this.position = position;
        }

        public double getValue() {
            return position;
        }
    }
}