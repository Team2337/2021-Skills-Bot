package frc.robot.nerdyfiles.swerve;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.*;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.ModulePosition;;

/**
 * Swerve Module Object used to run the calculations for the swerve drive
 * The swerve module uses joystick values from the command to change the
 * angle and drive positions
 * This Object uses the TalonFX's for both the angle motor and drive motor
 * Both will need to be passed in when the object is created
 * @see SwerveDriveCommand
 * @author Madison J. and Zayd A.
 * @category SWERVE
 */
public class FXSwerveModule {

    private ModulePosition position;

    /**
     * Proportional value for the drive motor speed
     * This is used to scale the error to a funcitonal speed for the motors
     */
    private static final double kDriveP = 0.1;

    /**
     * Integral value for the drive motor speed
     * This value is used to reduce oscillation when sending the motor to a setpoint
     */
    private static final double kDriveI = 0;

    /**
     * Derivative value for the drive motor speed
     * This is added to the speed of the motors to increase power at
     * smaller errors
     */
    private static final double kDriveD = 0;

    /**
     * Feet Forward value for the drive motor speed
     * Sets the error to be higher than the actual error
     * causing the motor to increase the power output to
     * be able to reach its setpoint
     */
    private static final double kDriveF = 0;

    private static final double kAngleP = 1.0;
    private static final double kAngleI = 0.0;
    private static final double kAngleD = 0.0;
    private static final double kAngleF = 0.00;
    private static final double kAngleAllowableClosedloopError = 5;
    private static final double kAngleOpenloopRamp = 0.15;

    /** TalonFX motor controller, used as an angle motor in the swerve module */
    private TalonFX driveMotor;

    /** TalonFX motor controller, used as a drive motor in the swerve module */
    private TalonFX angleMotor;

    /** CANCoder encoder, used measure the rotational position of the angle motor */
    private CANCoder canCoder;

    /**
     * 4in wheels on the MK3 Swerve modules
     * https://www.swervedrivespecialties.com/products/mk3-swerve-module
     */
    private static final double kWheelDiameterInches = 4;
    private static final double kDriveGearRatio = 8.16;

    /**
     * The number of ticks in a single revolution for the encoder being used
     * https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html#sensor-resolution
     */
    private static final double kAngleCANcoderTicksPerRotation = 4096;
    private static final double kDriveEncoderTicksPerRotation = 2048;

    // Inches traveled per tick can be found by using our circumference (2pi * r) of our wheel
    private static final double kWheelInchesPerRotation = 2 * Math.PI * (kWheelDiameterInches / 2);
    private static final double kWheelTicksPerRevolution = kDriveEncoderTicksPerRotation * kDriveGearRatio;
    private static final double kInchesPerTick = kWheelInchesPerRotation / kWheelTicksPerRevolution;
    private static final double kDriveFeetPerTick = kInchesPerTick / 12;

    /**
     * Swerve Module Object used to run the calculations for the swerve drive
     * The swerve module uses joystick values from the command to change the
     * angle and drive positions
     * This Object uses the TalonFX's for both the angle motor and drive motor
     * Both will need to be passed in when the object is created
     * @see SwerveDriveCommand
     * @param position - int Module ID to call each module (not CAN IDs)
     * @param driveMotor - TalonFX motor object with CAN ID of the module's drive motor
     * @param angleMotor - TalonFX motor object with CAN ID of the module's angle motor
     * @param canCoder - CANCoder sensor object that measures the rotation of the angle motor
     * @param angleMotorOffsetDegrees - The angle offset for the current module in degrees
     */
    public FXSwerveModule(Constants.Swerve.ModulePosition position, TalonFX driveMotor, TalonFX angleMotor, CANCoder canCoder, double angleMotorOffsetDegrees) {
        this.position = position;
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.canCoder = canCoder;

        TalonFXConfiguration angleTalonFXConfiguration = new TalonFXConfiguration();
        TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();

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

        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        //canCoderConfiguration.customParam0 = canCoder.configGetCustomParam(0);
        //canCoderConfiguration.magnetOffsetDegrees = canCoder.configGetCustomParam(0) + (position.value * 90);
        canCoderConfiguration.magnetOffsetDegrees = angleMotorOffsetDegrees + (position.value * 90);
        canCoder.configAllSettings(canCoderConfiguration);

        SmartDashboard.putNumber("Get Custom Param/" + canCoder.getDeviceID(), canCoder.configGetCustomParam(0));
        SmartDashboard.putNumber("Magnet Offest Degrees/" + canCoder.getDeviceID(), canCoder.configGetMagnetOffset());

        angleTalonFXConfiguration.slot0.kP = kAngleP;
        angleTalonFXConfiguration.slot0.kI = kAngleI;
        angleTalonFXConfiguration.slot0.kD = kAngleD;
        angleTalonFXConfiguration.slot0.kF = kAngleF;
        angleTalonFXConfiguration.slot0.allowableClosedloopError = kAngleAllowableClosedloopError;
        angleTalonFXConfiguration.openloopRamp = kAngleOpenloopRamp;

        // Use the CANCoder as the remote sensor for the primary TalonFX PID
        angleTalonFXConfiguration.remoteFilter0.remoteSensorDeviceID = canCoder.getDeviceID();
        angleTalonFXConfiguration.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        angleTalonFXConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;

        angleMotor.configAllSettings(angleTalonFXConfiguration);

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

        /* --- Talon FX Drive Configurations --- */
        driveTalonFXConfiguration.slot0.kP = kDriveP;
        driveTalonFXConfiguration.slot0.kI = kDriveI;
        driveTalonFXConfiguration.slot0.kD = kDriveD;
        driveTalonFXConfiguration.slot0.kF = kDriveF;
        /**
         * 0.5s seconds from neutral to full output
         * As a note - we're looking to do some other fixes to make teleop driving smoother.
         * This value might be able to be removed if we do smoothing pre-setting the percentage
         */
        driveTalonFXConfiguration.openloopRamp = 0.2;
        // driveTalonFXConfiguration.closedloopRamp = 0.2; // 0.2s seconds from neutral to full output
        // driveTalonFXConfiguration.slot0.allowableClosedloopError = 100;

        driveMotor.configAllSettings(driveTalonFXConfiguration);

        // Speeds are in ticks/100ms
        int kMaxVelocityTicks = 22627;

        /* --- Motion Magic --- */
        // Sets the velocity & accelaration for the motion magic mode
        driveMotor.configMotionCruiseVelocity(kMaxVelocityTicks , 0);
        driveMotor.configMotionAcceleration(kMaxVelocityTicks * 1, 0);

        // Sets how the motor will react when there is no power applied to the motor
        driveMotor.setNeutralMode(NeutralMode.Coast);

        // /* --- Setup Angle Current Limits --- */
        // StatorCurrentLimitConfiguration currentLimitConfigurationAngle = new StatorCurrentLimitConfiguration();
        // // Sets the current limit for the angle motor
        // currentLimitConfigurationAngle.currentLimit = 50;
        // // Enables the current limit
        // currentLimitConfigurationAngle.enable = true;
        // // Sets the minimum current reading needed in order to initiate the current reading
        // currentLimitConfigurationAngle.triggerThresholdCurrent = 40;
        // // Sets the minimum amount of time beyond the trigger threshold reading needed in order to initiate the current reading
        // currentLimitConfigurationAngle.triggerThresholdTime = 3;

        // /* --- Setup Drive Current Limit --- */
        // StatorCurrentLimitConfiguration currentLimitConfigurationDrive = new StatorCurrentLimitConfiguration();
        // // Sets the current limit for the drive motor
        // currentLimitConfigurationDrive.currentLimit = 50;
        // // Enables the current limit
        // currentLimitConfigurationDrive.enable = true;
        // // Sets the minimum current reading needed in order to initiate the current reading
        // currentLimitConfigurationDrive.triggerThresholdCurrent = 40;
        // // Sets the minimum amount of time beyond the trigger threshold reading needed in order to initiate the current reading
        // currentLimitConfigurationDrive.triggerThresholdTime = 3;

        // /* --- Set Amperage Limits --- */
        // angleMotor.configStatorCurrentLimit(currentLimitConfigurationAngle, 0);
        // driveMotor.configStatorCurrentLimit(currentLimitConfigurationDrive, 0);
    }

    /**
     * Gets the module number of the current module
     * @return The module number of the current module
     */
    public int getModuleNumber() {
        return position.value;
    }

    /**
     * Gets the rotational position of the module
     * @return The rotational position of the angle motor in degrees
     */
    public double getAngle() {
        return canCoder.getAbsolutePosition();
    }

    /**
     * Get the velocity of the drive motor for the module.
     * @return The velocity for the drive motor of the module in feet per second.
     */
    private double getVelocity() {
        // Ticks per 100ms
        double velocityTicks = driveMotor.getSelectedSensorVelocity();

        // Inches per 100ms
        double velocityInches = velocityTicks * kInchesPerTick;

        // Inches -> Feet (12 inches per foot)
        double velocityFeet = velocityInches / 12;
        // 100ms -> 1s (10 100ms intervals per second)
        return velocityFeet * 10;
    }

    /**
     * Set the speed + rotation of the swerve module from a SwerveModuleState object
     * @param desiredState - A SwerveModuleState representing the desired new state of the module
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean shouldUpdateAngle, boolean isJoystickControl) {
        Rotation2d currentRotation = Rotation2d.fromDegrees(getAngle());
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);

        if (shouldUpdateAngle) {
            setAngle(state.angle);
        }

        double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond);
        if(isJoystickControl) {
            driveMotor.set(TalonFXControlMode.PercentOutput, feetPerSecond / Constants.Swerve.MAX_FEET_PER_SECOND);
        } else {
            driveMotor.set(TalonFXControlMode.Velocity, ((feetPerSecond / 10) * 12) / kInchesPerTick);
        }
    }

    public void setMotionMagic(Rotation2d angle, double distanceFeet) {
        Rotation2d currentRotation = Rotation2d.fromDegrees(getAngle());
        SwerveModuleState state = SwerveModuleState.optimize(new SwerveModuleState(1, angle), currentRotation);

        setAngle(state.angle);

        driveMotor.set(TalonFXControlMode.MotionMagic, distanceFeet / kDriveFeetPerTick);
    }

    private void setAngle(Rotation2d angle) {
        Rotation2d currentRotation = Rotation2d.fromDegrees(getAngle());

        // Find the rotational difference between the current state and the desired state
        // Will be a positive value for clockwise rotations, neg for ccw rotations
        Rotation2d rotationDelta = angle.minus(currentRotation);

        double deltaTicks = (rotationDelta.getDegrees() / 360) * kAngleCANcoderTicksPerRotation;
        double currentTicks = canCoder.getPosition() / canCoder.configGetFeedbackCoefficient();
        double desiredTicks = currentTicks + deltaTicks;
        // Set the absolute position for the motor by converting our degrees to # of ticks for the rotational value
        angleMotor.set(TalonFXControlMode.Position, desiredTicks);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(Units.feetToMeters(getVelocity()), Rotation2d.fromDegrees(getAngle()));
    }

    public void resetDriveMotor() {
        driveMotor.setSelectedSensorPosition(0);
    }

    public void resetAngleMotor() {
        setAngle(new Rotation2d(0));
    }

    public void stopAngleMotor() {
        angleMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void stopDriveMotor() {
        driveMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    /**
     * Gets the angle motor temperature
     * @return - The temperature of the angle motors
     */
    private double getAngleMotorTemperature() {
        return angleMotor.getTemperature();
    }

    /**
     * Gets the drive motor temperature
     * @return - The temperature of the drive motors
     */
    private double getDriveMotorTemperature() {
        return driveMotor.getTemperature();
    }

    public double getDriveMotorPosition() {
        return driveMotor.getSelectedSensorPosition();
    }

    public void playNote() {
        angleMotor.set(TalonFXControlMode.MusicTone, 261.626); // C4
        driveMotor.set(TalonFXControlMode.MusicTone, 261.626); // C4
    }

    public void stopNote() {
        angleMotor.set(TalonFXControlMode.MusicTone, 0);
        driveMotor.set(TalonFXControlMode.MusicTone, 0);
    }

    public void logDebug() {
        SmartDashboard.putNumber("Module Angle (Degrees)/" + getModuleNumber(), getAngle());
        SmartDashboard.putNumber("Angle Motor Temperature/" + getModuleNumber(), getAngleMotorTemperature());
        SmartDashboard.putNumber("Drive Motor Temperature/" + getModuleNumber(), getDriveMotorTemperature());
        SmartDashboard.putNumber("Inches Traveled/" + getModuleNumber(), kInchesPerTick * getDriveMotorPosition());
        SmartDashboard.putNumber("Feet Traveled/" + getModuleNumber(), kDriveFeetPerTick * getDriveMotorPosition());
        SmartDashboard.putNumber("Drive Encoders/" + getModuleNumber(), getDriveMotorPosition());
        SmartDashboard.putNumber("Velocity/" + getModuleNumber(), getVelocity());
    }

}
