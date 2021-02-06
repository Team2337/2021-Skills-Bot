package frc.robot.nerdyfiles.swerve;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Swerve Module Object used to run the calculations for the swerve drive
 * The swerve module uses joystick values from the command to change the
 * angle and drive positions
 * This Object uses the TalonFX's for both the angle motor and drive motor
 * Both will need to be passed in when the object is created
 * @see SwerveDriveCommand
 * @author Bryce G.
 * @category SWERVE
 */
public class FXSwerveModule {

    /* --- Ints --- */

    /** The current module's ID number (0 -> 3)*/
    private int moduleNumber;

    /* --- Doubles --- */

    /** The angle offset from the zero position on the angle motor in RADIANS */
    private double angleMotorOffset;

    /**
     * The error of the previous iteration of the angle calculations
     * This is requred in order to find the D (derivative) value for
     * the speed calculations
     */
    private double lastError = 0;

    /**
     * The allowable angle slop on the modules in degrees
     * This will reduce the oscillation on the angle motor
     * when within +- 3 degrees of the target
     */
    private double allowableErrorDegree = 3;

    /**
     * Proportional value for the drive motor speed
     * This is used to scale the error to a funcitonal speed for the motors
     */
    private double driveP = 15;

    /**
     * Integral value for the drive motor speed
     * This value is used to reduce oscillation when sending the motor to a setpoint
     */
    private double driveI = 0.01;

    /**
     * Derivative value for the drive motor speed
     * This is added to the speed of the motors to increase power at
     * smaller errors
     */
    private double driveD = 0.1;

    /**
     * Feet Forward value for the drive motor speed
     * Sets the error to be higher than the actual error
     * causing the motor to increase the power output to
     * be able to reach its setpoint
     */
    private double driveF = 0.2;

    /**
     * Proportional value for the angle motor speed
     * This is used to scale the error to a funcitonal speed for the motors
     */
    private double angleP = 0.63;

    /**
     * Derivative value for the angle motor speed
     * This is added to the speed of the motors to increase power at
     * smaller errors
     */
    private double angleD = 0.02;

    /** Sets the max speed for the drive motors */
    private double driveMaxSpeed = 1.0;

    private int angleAllowableClosedloopError = 5;
    private double talonAngleP = 2.5;
    private double talonAngleI = 0;
    private double talonAngleD = 0;
    private double talonAngleF = 0;

    /* --- Booleans --- */

    /** Sets the inversion mode on the drive motors (True: invered | False: not inverted) */
    private boolean isDriveInverted = false;

    /* --- Motor Controllers --- */

    /** TalonFX motor controller, used as an angle motor in the swerve module */
    public TalonFX driveMotor;

    /** TalonFX motor controller, used as a drive motor in the swerve module */
    public TalonFX angleMotor;

    /* --- Sensors --- */

    /**
     * Analog potentiometer used to measure the exact location of
     * each swerve module. The output recieved from this is in voltage (0 -> 5)
     */
    public AnalogInput analogAngleSensor;
    public CANCoder CANAngleSensor;

    /* --- Current Limit Stator --- */
    private StatorCurrentLimitConfiguration currentLimitConfigurationAngle = new StatorCurrentLimitConfiguration();
    private StatorCurrentLimitConfiguration currentLimitConfigurationDrive = new StatorCurrentLimitConfiguration();

    /* --- Talon FX Configurations --- */
    private TalonFXConfiguration TalonFXConfigurationAngle;
    private TalonFXConfiguration TalonFXConfigurationDrive;



    /**
     * Swerve Module Object used to run the calculations for the swerve drive
     * The swerve module uses joystick values from the command to change the
     * angle and drive positions
     * This Object uses the TalonFX's for both the angle motor and drive motor
     * Both will need to be passed in when the object is created
     * @see SwerveDriveCommand
     * @param moduleNumber - int Module ID to call each module (not CAN IDs)
     * @param driveMotor - TalonFX motor Object with CAN ID of the module's drive motor
     * @param angleMotor - TalonFX motor Object with CAN ID of the module's angle motor
     * @param angleMotorOffset - double value indicating the angle offset for the current module
     * @param analogAngleSensor - AnalogInput sensor Object with the Analog Port of the current module
     */
    public FXSwerveModule(int moduleNumber, TalonFX driveMotor, TalonFX angleMotor, double angleMotorOffset, AnalogInput analogAngleSensor) {
        this.moduleNumber = moduleNumber;
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.angleMotorOffset = angleMotorOffset;
        this.analogAngleSensor = analogAngleSensor;
        TalonFXConfigurationDrive = new TalonFXConfiguration();
        TalonFXConfigurationAngle = new TalonFXConfiguration();

        /* --- Set Factory Default --- */

        // Resets the angle motor to its factory default
        angleMotor.configFactoryDefault();
        // Resets the drive motor to its factory default
        driveMotor.configFactoryDefault();

        /*****************************/
        /* ------------------------- */
        /* --- Angle Motor Setup --- */
        /* ------------------------- */
        /*****************************/

        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        angleMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        angleMotor.setNeutralMode(NeutralMode.Coast);
        angleMotor.configOpenloopRamp(0.1);
        angleMotor.setSensorPhase(false);
        angleMotor.setInverted(false);

        TalonFXConfigurationAngle.slot0.kP = talonAngleP;
        TalonFXConfigurationAngle.slot0.kI = talonAngleI;
        TalonFXConfigurationAngle.slot0.kD = talonAngleD;
        TalonFXConfigurationAngle.slot0.kF = talonAngleF;
        TalonFXConfigurationAngle.slot0.allowableClosedloopError = angleAllowableClosedloopError;
        TalonFXConfigurationAngle.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        TalonFXConfigurationAngle.feedbackNotContinuous = true;
        TalonFXConfigurationAngle.openloopRamp = 0.15;

        angleMotor.configAllSettings(TalonFXConfigurationAngle);
        /*****************************/
        /* ------------------------- */
        /* --- Drive Motor Setup --- */
        /* ------------------------- */
        /*****************************/

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        driveMotor.configClosedloopRamp(0.1);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);

        /* --- Drive PID --- */
        driveMotor.config_kP(0, driveP, 0);
        driveMotor.config_kI(0, driveI, 0);
        driveMotor.config_kD(0, driveD, 0);
        driveMotor.config_kF(0, driveF, 0);

        /* --- Talon FX Drive Configurations --- */
        TalonFXConfigurationDrive.slot0.kP = driveP;
        TalonFXConfigurationDrive.slot0.kI = driveI;
        TalonFXConfigurationDrive.slot0.kD = driveD;
        TalonFXConfigurationDrive.slot0.kF = driveF;
        TalonFXConfigurationDrive.peakOutputForward = driveMaxSpeed;
        TalonFXConfigurationDrive.peakOutputReverse = -driveMaxSpeed;
        TalonFXConfigurationDrive.slot0.allowableClosedloopError = 100;
        TalonFXConfigurationDrive.closedloopRamp = 0.55;
        TalonFXConfigurationDrive.openloopRamp = 0.2;

        driveMotor.configAllSettings(TalonFXConfigurationDrive);

        /* --- Motion Magic --- */
        // Sets the velocity & accelaration for the motion magic mode
        driveMotor.configMotionCruiseVelocity(640, 0);
        driveMotor.configMotionAcceleration(200, 0);

        // Sets how the motor will react when there is no power applied to the motor
        driveMotor.setNeutralMode(NeutralMode.Coast);

        /* --- Setup Angle Current Limits --- */
        // Sets the current limit for the angle motor
        currentLimitConfigurationAngle.currentLimit = 50;
        // Enables the current limit
        currentLimitConfigurationAngle.enable = true;
        // Sets the minimum current reading needed in order to initiate the current reading
        currentLimitConfigurationAngle.triggerThresholdCurrent = 40;
        // Sets the minimum amount of time beyond the trigger threshold reading needed in order to initiate the current reading
        currentLimitConfigurationAngle.triggerThresholdTime = 3;

        /* --- Setup Drive Current Limit --- */
        // Sets the current limit for the drive motor
        currentLimitConfigurationDrive.currentLimit = 50;
        // Enables the current limit
        currentLimitConfigurationDrive.enable = true;
        // Sets the minimum current reading needed in order to initiate the current reading
        currentLimitConfigurationDrive.triggerThresholdCurrent = 40;
        // Sets the minimum amount of time beyond the trigger threshold reading needed in order to initiate the current reading
        currentLimitConfigurationDrive.triggerThresholdTime = 3;

        /* --- Set Amperage Limits --- */
        angleMotor.configStatorCurrentLimit(currentLimitConfigurationAngle, 0);
        driveMotor.configStatorCurrentLimit(currentLimitConfigurationDrive, 0);
    }

    public FXSwerveModule(int moduleNumber, TalonFX driveMotor, TalonFX angleMotor, double angleMotorOffset, CANCoder CANAngleSensor) {
        this.moduleNumber = moduleNumber;
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.angleMotorOffset = angleMotorOffset;
        this.CANAngleSensor = CANAngleSensor;
        TalonFXConfigurationDrive = new TalonFXConfiguration();
        TalonFXConfigurationAngle = new TalonFXConfiguration();

        /* --- Set Factory Default --- */

        // Resets the angle motor to its factory default
        angleMotor.configFactoryDefault();
        // Resets the drive motor to its factory default
        driveMotor.configFactoryDefault();

        /****************************************/
        /* ------------------------------------ */
        /* --- Angle Motor And Sensor Setup --- */
        /* ------------------------------------ */
        /****************************************/

        CANCoderConfiguration CANCoderConfigurationAngle = new CANCoderConfiguration();
        CANAngleSensor.configAllSettings(CANCoderConfigurationAngle);


        TalonFXConfigurationAngle.slot0.kP = talonAngleP;
        TalonFXConfigurationAngle.slot0.kI = talonAngleI;
        TalonFXConfigurationAngle.slot0.kD = talonAngleD;
        TalonFXConfigurationAngle.slot0.kF = talonAngleF;
        TalonFXConfigurationAngle.slot0.allowableClosedloopError = angleAllowableClosedloopError;
        TalonFXConfigurationAngle.feedbackNotContinuous = true;
        TalonFXConfigurationAngle.openloopRamp = 0.15;

        TalonFXConfigurationAngle.remoteFilter0.remoteSensorDeviceID = CANAngleSensor.getDeviceID();
        TalonFXConfigurationAngle.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        TalonFXConfigurationAngle.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;

        angleMotor.configAllSettings(TalonFXConfigurationAngle);

        angleMotor.setNeutralMode(NeutralMode.Coast);

        /*****************************/
        /* ------------------------- */
        /* --- Drive Motor Setup --- */
        /* ------------------------- */
        /*****************************/

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        driveMotor.configClosedloopRamp(0.1);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);

        /* --- Drive PID --- */
        driveMotor.config_kP(0, driveP, 0);
        driveMotor.config_kI(0, driveI, 0);
        driveMotor.config_kD(0, driveD, 0);
        driveMotor.config_kF(0, driveF, 0);

        /* --- Talon FX Drive Configurations --- */
        TalonFXConfigurationDrive.slot0.kP = driveP;
        TalonFXConfigurationDrive.slot0.kI = driveI;
        TalonFXConfigurationDrive.slot0.kD = driveD;
        TalonFXConfigurationDrive.slot0.kF = driveF;
        TalonFXConfigurationDrive.peakOutputForward = driveMaxSpeed;
        TalonFXConfigurationDrive.peakOutputReverse = -driveMaxSpeed;
        TalonFXConfigurationDrive.slot0.allowableClosedloopError = 100;
        TalonFXConfigurationDrive.closedloopRamp = 0.55;
        TalonFXConfigurationDrive.openloopRamp = 0.2;

        driveMotor.configAllSettings(TalonFXConfigurationDrive);

        /* --- Motion Magic --- */
        // Sets the velocity & accelaration for the motion magic mode
        driveMotor.configMotionCruiseVelocity(640, 0);
        driveMotor.configMotionAcceleration(200, 0);

        // Sets how the motor will react when there is no power applied to the motor
        driveMotor.setNeutralMode(NeutralMode.Coast);

        /* --- Setup Angle Current Limits --- */
        // Sets the current limit for the angle motor
        currentLimitConfigurationAngle.currentLimit = 50;
        // Enables the current limit
        currentLimitConfigurationAngle.enable = true;
        // Sets the minimum current reading needed in order to initiate the current reading
        currentLimitConfigurationAngle.triggerThresholdCurrent = 40;
        // Sets the minimum amount of time beyond the trigger threshold reading needed in order to initiate the current reading
        currentLimitConfigurationAngle.triggerThresholdTime = 3;

        /* --- Setup Drive Current Limit --- */
        // Sets the current limit for the drive motor
        currentLimitConfigurationDrive.currentLimit = 50;
        // Enables the current limit
        currentLimitConfigurationDrive.enable = true;
        // Sets the minimum current reading needed in order to initiate the current reading
        currentLimitConfigurationDrive.triggerThresholdCurrent = 40;
        // Sets the minimum amount of time beyond the trigger threshold reading needed in order to initiate the current reading
        currentLimitConfigurationDrive.triggerThresholdTime = 3;

        /* --- Set Amperage Limits --- */
        angleMotor.configStatorCurrentLimit(currentLimitConfigurationAngle, 0);
        driveMotor.configStatorCurrentLimit(currentLimitConfigurationDrive, 0);
    }

    /**************************/
    /* ---------------------- */
    /* --- Sensor Methods --- */
    /* ---------------------- */
    /**************************/

    /**
     * Gets the angle voltages
     * @return - The angle voltages
     */
    public double getAnalogVoltage() {
        return analogAngleSensor.getVoltage();
    }

    public double getCANCoderDegrees() {
        return CANAngleSensor.getAbsolutePosition();
    }

    public double getCANCoderRadians() {
        SmartDashboard.putNumber("Absolute Position 2" + moduleNumber, CANAngleSensor.getAbsolutePosition()* (Math.PI/180));
        return CANAngleSensor.getAbsolutePosition() * (Math.PI/180);
    }

    /**
     * Gets the raw analog input, and divides it by the current 5V reading from
     * the robot to normalize the sensor value in terms of (0 -> 1)
     * @return - double sensor positional value from (0 -> 1)
     */

    public double getNormalizedAnalogVoltage() {
        double voltage = 0;
        if(Constants.Swerve.ANALOGENCODER) {
             voltage = analogAngleSensor.getVoltage() / RobotController.getVoltage5V();
        }
        return voltage;
    }

    /**
     * Takes the normalized sensor value, converting it to RADIANS
     * @return - double angle value in RADIANS
     */
    public double getNormalizedAnalogVoltageRadians() {
        return getNormalizedAnalogVoltage() * (2 * Math.PI);
    }

    public double getRadians() {
        double radians;
        if (Constants.Swerve.ANALOGENCODER) {
            radians = getNormalizedAnalogVoltageRadians();
        } else {
            radians = getCANCoderRadians();
        }
        SmartDashboard.putNumber("Radians 2" + moduleNumber, radians);
        return radians;
    }

    /*************************/
    /* --------------------- */
    /* --- Angle Methods --- */
    /* --------------------- */
    /*************************/

    /**
     * Adds the offset to the angle motors
     * @return - Double value adds the angle offset
     */
    public double adjustAngleWithOffset() {
        return (getRadians() + this.angleMotorOffset) % (2 * Math.PI);
    }

    /**
     * Takes the desired angle, and the current angle and computes the delta (current - target)
     * to set the speed to the angle motors to move the module to the
     * desired angle, without overshooting.
     * @param targetAngle - double value in degrees
     */
    public void setModuleAngle(double targetAngle) {
        double delta = targetAngle - getCANCoderDegrees();
        double angle;
        if (Math.abs(delta) > 90.0) {
            driveMotor.setInverted(true);
            Rotation2d rotated = Rotation2d.fromDegrees(targetAngle).rotateBy(Rotation2d.fromDegrees(180.0));
            angle = rotated.getDegrees();
        } else {
            driveMotor.setInverted(false);
            angle = targetAngle;
        }

        angleMotor.set(TalonFXControlMode.Position,  (angle/360) * 4096);

    }

    /**
     * Sets the speed of the module angle motor
     * @param speed - double value to set the speed to the angle motor (-1 -> 1)
     */
    public void setAngleMotorSpeed(double speed) {
        angleMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Gets the angle offset on the module (-2PI -> 2PI)
     * @return - double value of the angle offset of the current module (-2PI -> 2PI)
     */
    public double getAngleOffset() {
        return this.angleMotorOffset;
    }

    /**
     * Gets the angle motor temperature
     * @return - The temperature of the angle motors
     */
    public double getAngleMotorTemperature() {
        return angleMotor.getTemperature();
    }

    /**
     * Gets the drive encoder position in ticks
     * @return - The selected sensor position
     */
    public double getDriveEncoderValue() {
        return driveMotor.getSelectedSensorPosition(0);
    }

    /**
     * Sets the encoder drive position
     * @param position - Sets the selected sensor position
     */
    public void setDriveEncoder(int position) {
        driveMotor.setSelectedSensorPosition(position, 0, 0);
    }

    /**
     * Zeros all drive encoders
     */
    public void zeroDriveEncoder() {
        setDriveEncoder(0);
    }

    /**
     * Gets the angle encoder position in ticks
     * @return - The selected sensor position
     */
    public double getAngleEncoderValue() {
        return angleMotor.getSelectedSensorPosition(0);
    }

    public void setAngleEncoder(int position) {
        angleMotor.setSelectedSensorPosition(position, 0, 0);
    }

    /*************************/
    /* --------------------- */
    /* --- Drive Methods --- */
    /* --------------------- */
    /*************************/

    /**
     * Sets the drive to be inverted
     * @param isDriveInverted - boolean value stating the drive inversion mode (True: inverted | False: not inverted)
     */
    public void setDriveInverted(boolean isDriveInverted) {
        this.isDriveInverted = isDriveInverted;
    }

    /**
     * Sets the speed of the drive motors
     * @param speed - double value in percent of the motors (-1 -> 1)
     */
    public void setDriveSpeed(double speed) {
        if(isDriveInverted) speed = -speed;

        driveMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Gets the drive motor temperature
     * @return - The temperature of the drive motors
     */
    public double getDriveMotorTemperature() {
        return driveMotor.getTemperature();
    }
}