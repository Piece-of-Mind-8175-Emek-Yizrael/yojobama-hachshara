// RobotBuilder Version: 6.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Robot.

package pom.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.POM_lib.Joysticks.PomXboxController;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in
 * the project.
 */
public class Robot extends TimedRobot {

    private final CANSparkMax liftMotor = new CANSparkMax(Constants.ArmConstants.LIFT_MOTOR_PORT, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    boolean complexArm = false;
    DifferentialDrive driveTrain;
    WPI_TalonSRX rightDriveTalon = new WPI_TalonSRX(Constants.DriveConstants.RIGHT_TALON);
    WPI_VictorSPX rightDriveVictor = new WPI_VictorSPX(Constants.DriveConstants.RIGHT_VICTOR);
    WPI_TalonSRX leftDriveTalon = new WPI_TalonSRX(Constants.DriveConstants.LEFT_TALON);
    WPI_VictorSPX leftDriveVictor = new WPI_VictorSPX(Constants.DriveConstants.LEFT_VICTOR);
    DigitalInput fold = new DigitalInput(Constants.ArmConstants.FOLD_SWICH_PORT);
    DigitalInput ground = new DigitalInput(Constants.ArmConstants.GROUND_SWICH_PORT);
    CANSparkMax intakeMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_PORT, MotorType.kBrushless);
    PomXboxController driverController = new PomXboxController(Constants.DRIVER_CONTROLLER_PORT);
    PigeonIMU imu = new PigeonIMU(Constants.DriveConstants.PigeonIMU);
    AnalogPotentiometer sensor = new AnalogPotentiometer(0);
    int timesTurnd = 0;
    double startAngle;
    Timer timer = new Timer();
    AutonomousStage autoStage = AutonomousStage.DRIVE_FORWARD;
    boolean isAuto = false;
    IntakeState intakeState = IntakeState.FREE;
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private RelativeEncoder encoder = liftMotor.getEncoder();
    private ArmFeedforward ff = new ArmFeedforward(0, Constants.ArmConstants.FF_KG, 0);

    @Override
    public void robotInit() {

        //intakeMotor.setSecondaryCurrentLimit(Constants.INTAKE_CURRENT_LIMIT);

        rightDriveVictor.follow(rightDriveTalon);
        leftDriveVictor.follow(leftDriveTalon);

        rightDriveTalon.setInverted(true);
        rightDriveVictor.setInverted(true);
        leftDriveTalon.setInverted(false);
        leftDriveVictor.setInverted(false);

        driveTrain = new DifferentialDrive(leftDriveTalon::set, rightDriveTalon::set);
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = RobotContainer.getInstance();
        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder);
        enableLiveWindowInTest(true);
        encoder.setPositionConversionFactor((1.0 / 50) * (16.0 / 42) * 2 * Math.PI);
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        //    System.err.println("I am a null and I am ok. I bully Uri most of the day");
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        timer.stop();

        autoStage = AutonomousStage.PUT_CUBE;
        isAuto = true;
        startAngle = imu.getYaw();
    }

    @Override
    public void teleopInit() {

        isAuto = false;
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }    ArmState armState = ArmState.FREE, lastArmState = armState;

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */

    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("ultrasonic", sensor.get());
        SmartDashboard.putNumber("arm encoder", encoder.getPosition());
        SmartDashboard.putNumber("gravity resist", resistGravity());
        SmartDashboard.putBoolean("ground switch", !ground.get());
        SmartDashboard.putBoolean("fold switch", !fold.get());
        if (!fold.get()) {
            encoder.setPosition(-0.323);
        }
        driveTrain.feed();
    }

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */


    public double resistGravity() {
        return ff.calculate(encoder.getPosition(), 0);
    }

    @Override
    public void disabledPeriodic() {
    }

    /*public void doIntake()
    {
        if((isYPressed == null) && (isAPressed == null))
        {
            if(driverController.leftTrigger().getAsBoolean()) diraction=1;
            else if(driverController.rightTrigger().getAsBoolean()) diraction=-1;
            else diraction=0;
        }

        if(diraction != lastDiraction)
        {
            motor.set(Constants.INTAKE_SPIN_POWER * Math.signum(diraction));    
        }
    }*/

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

        SmartDashboard.putString("Autonomus stage", autoStage.toString());
        SmartDashboard.putNumber("timer time", timer.get());

        if (autoStage.stage < 6) {
            switch (autoStage) {
                case PUT_CUBE:
                    if (encoder.getPosition() < Constants.ArmConstants.ARM_ENCODER_POSITION_HIGH) {
                        armState = ArmState.OUTOPEN;
                        timer.restart();
                    } else if (timer.get() > 1) {
                        armState = ArmState.HOLD;
                        intakeMotor.set(0);
                        autoStage = AutonomousStage.TURN_TO_CONE;
                    } else {
                        //timer.restart();
                        armState = ArmState.HOLD;
                        intakeMotor.set(Constants.IntakeConstants.INTAKE_SPIN_POWER_OUT);
                    }
                    break;

                case TURN_TO_CONE:
                    if (Math.abs(imu.getYaw() - startAngle) < 174 && armState == ArmState.HOLD)
                        driveTrain.arcadeDrive(0, -0.15, false);
                    else if (Math.abs(imu.getYaw() - startAngle) >= 174) {
                        driveTrain.arcadeDrive(0, 0, false);
                        armState = ArmState.NOPEN;
                        autoStage = AutonomousStage.DRIVE_FORWARD;
                        timer.restart();
                    }
                    break;
                case DRIVE_FORWARD:
                    if (armState != ArmState.FREE) {
                        if (!ground.get()) armState = ArmState.FREE;
                        timer.restart();
                    } else if (timer.get() < Constants.TIME_THAT_URI_WANTS && armState == ArmState.FREE) {
                        driveTrain.arcadeDrive(0.25, 0, false);
                        intakeMotor.set(Constants.IntakeConstants.INTAKE_SPIN_POWER_IN);
                    } else if (timer.get() >= Constants.TIME_THAT_URI_WANTS) {
                        intakeMotor.set(0);
                        armState = ArmState.CLOSE;
                        driveTrain.arcadeDrive(0, 0, false);
                        intakeState = IntakeState.FREE;
                        autoStage = AutonomousStage.TURN_TO_DEPOSIT;
                        startAngle = imu.getYaw();
                    }
                    break;
                case TURN_TO_DEPOSIT:
                    if (Math.abs(imu.getYaw() - startAngle) < 180 && armState == ArmState.FREE)
                        driveTrain.arcadeDrive(0, 0.15, false);
                    else if (Math.abs(imu.getYaw() - startAngle) >= 180) {
                        timer.restart();
                        driveTrain.arcadeDrive(0, 0, false);
                        autoStage = AutonomousStage.DRIVE_TO_DEPOSIT;
                    }
                    break;
                case DRIVE_TO_DEPOSIT:
                    if (timer.get() < Constants.TIME_THAT_URI_WANTS) driveTrain.arcadeDrive(0.2, 0, false);
                    else {
                        timer.reset();
                        armState = ArmState.OUTOPEN;
                        autoStage = AutonomousStage.PUT_CONE;
                        driveTrain.arcadeDrive(0, 0, false);
                    }
                    break;
                case PUT_CONE:
                    if (armState != ArmState.HOLD) {
                        timer.restart();
                        if (encoder.getPosition() < Constants.ArmConstants.ARM_ENCODER_POSITION_HIGH)
                            armState = ArmState.OUTOPEN;
                        else armState = ArmState.HOLD;
                    }
                    if (armState == ArmState.HOLD && timer.get() < 1)
                        intakeMotor.set(-Constants.IntakeConstants.INTAKE_SPIN_POWER_IN);
                    else if (timer.get() >= 1) {
                        armState = ArmState.CLOSE;
                        intakeMotor.set(0);
                        autoStage.stage++;
                    }
                    break;

                default:
                    isAuto = false;
                    break;
            }

        }

        doLift();
    }

    public void doLift() {
        if (true) {
            // checking if a button was pressed
            //----------------------------------------------------------------
            if (driverController.PovUp().getAsBoolean()) armState = ArmState.HOLD;
            if (driverController.aPressed().getAsBoolean()) armState = ArmState.CLOSE;
            if (driverController.yPressed().getAsBoolean()) armState = ArmState.NOPEN;

            if (driverController.bPressed().getAsBoolean()) armState = ArmState.INOPEN;
            if (driverController.bReleased().getAsBoolean()) {
                armState = ArmState.CLOSE;
                intakeState = IntakeState.FREE;
                complexArm = false;
            }
            if (driverController.xPressed().getAsBoolean()) armState = ArmState.OUTOPEN;
            if (driverController.xReleased().getAsBoolean()) {
                armState = ArmState.CLOSE;
                intakeState = IntakeState.FREE;
                complexArm = false;
            }
            //----------------------------------------------------------------
        }
        // checking if one of the switches is pressed
        //----------------------------------------------------------------
        if (!ground.get()) {
            if (lastArmState == ArmState.INOPEN) {
                complexArm = true;
                intakeState = IntakeState.IN;
                armState = ArmState.FREE;
            } else if (lastArmState == ArmState.OUTOPEN) {
                complexArm = true;
                intakeState = IntakeState.OUT;
                armState = ArmState.FREE;
            }
            if (armState == ArmState.NOPEN) armState = ArmState.FREE;
            else if (armState == ArmState.HOLD) armState = ArmState.FREE;
        } else if (ground.get() && (armState == ArmState.INOPEN || armState == ArmState.OUTOPEN)) complexArm = false;
        else if (!fold.get() && armState == ArmState.CLOSE) armState = ArmState.FREE;
        else if (!fold.get() && armState == ArmState.HOLD) armState = ArmState.FREE;
        //----------------------------------------------------------------

        armState.setResistGravity(resistGravity());
        liftMotor.set(armState.getSpeed());
    }

    /**
     * This function is called periodically during operator control.
     */

    @Override
    public void teleopPeriodic() {
        drive();
        doLift();
        doIntake();

        lastArmState = armState;
    }

    public void drive() {
        driveTrain.arcadeDrive(driverController.getLeftY() / 2, driverController.getRightX() / 2);
    }

    public void doIntake() {
        if (!complexArm) {
            if ((armState != ArmState.FREE) && (armState != ArmState.HOLD)) intakeState = IntakeState.FREE;

            else if (driverController.LB().getAsBoolean()) intakeState = IntakeState.IN;
            else if (driverController.RB().getAsBoolean()) intakeState = IntakeState.OUT;

            else if (!driverController.LB().getAsBoolean()) intakeState = IntakeState.FREE;
            else if (!driverController.RB().getAsBoolean()) intakeState = IntakeState.FREE;
        }
        intakeMotor.set(intakeState == IntakeState.OUT ? intakeState.getSpeedOUT() : intakeState.getSpeedIN());
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }


    public enum AutonomousStage {
        PUT_CUBE(0), TURN_TO_CONE(1), DRIVE_FORWARD(2), TURN_TO_DEPOSIT(3), DRIVE_TO_DEPOSIT(4), PUT_CONE(5);

        public int stage = 0;

        private AutonomousStage(int stage) {
            this.stage = stage;
        }

        public int get() {
            return stage;
        }
    }

    public enum IntakeState {
        IN(1), OUT(-1), FREE(0);

        int multiplier;

        private IntakeState(int multiplier) {
            this.multiplier = multiplier;
        }

        public double getSpeedIN() {
            return multiplier * Constants.IntakeConstants.INTAKE_SPIN_POWER_IN;
        }

        public double getSpeedOUT() {
            return multiplier * Constants.IntakeConstants.INTAKE_SPIN_POWER_OUT;
        }
    }

    public enum ArmState {
        NOPEN(1), CLOSE(-1), HOLD(0), FREE(true), INOPEN(1), OUTOPEN(1);

        int multiplier;
        double resistGravity;
        boolean isFree;

        private ArmState(int multiplier) {
            this.multiplier = multiplier;
            this.isFree = false;
        }

        private ArmState(boolean isFree) {
            this.multiplier = 0;
            this.isFree = true;
        }

        public void setResistGravity(double resistGravity) {
            this.resistGravity = resistGravity;
        }

        public double getSpeed() {
            return !isFree ? ((multiplier * Constants.ArmConstants.ARM_POWER) + resistGravity) : 0;
        }
    }



}