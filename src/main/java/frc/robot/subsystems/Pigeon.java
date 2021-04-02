package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Gyro class for CTRE Pigeon Gyro
 * @category CHASSIS
 * @author Team2337 - EngiNERDs
 * Must be initiated after Subsystems
 */
public class Pigeon extends SubsystemBase {

	/**
	 * Specifies whether or not the Pigeon will be in configuration mode.
	 */
	private final boolean configMode = false;

	/**
	 * Specifies whether or not the Pigeon will be in debug mode.
	 * @see #periodic()
	 */
	private final boolean pigeonDebug = false;

	/**
	 * Pigeon IMU object
	 */
	public static PigeonIMU pidgey;
	/**
	 * Data object for holding fusion information.
	 */
	public static PigeonIMU.FusionStatus gyrofusionStatus;
	/**
	 * Used to set the calibration of the gyro
	 */
	public static PigeonIMU.GeneralStatus gyroGenStatus;
	/**
	 * Array for Yaw Pitch and Roll values in degrees
	 */
	public double[] ypr_deg;
	/**
	 * Array for raw gyro values (x: roll | y: pitch | z: yaw)
	 */
	public double[] xyz_dps;
	/**
	 * Array for raw acceleration values
	 */
	public short[] xyz_accl;
	/**
	 * Timeout to set the yaw
	 */
	private int timeoutMs = 20;

	/**
	 * The subsystem to calibrate the Pigeon (gyro)
	 */
	public Pigeon() {
		pidgey = new PigeonIMU(0);

		//Change the configMode variable to true to calibrate the pigeon to the correct degree mode
		if(configMode){
			pidgey.configFactoryDefault();
			pidgey.enterCalibrationMode(CalibrationMode.BootTareGyroAccel, 10);
		}

		gyrofusionStatus = new PigeonIMU.FusionStatus();
		gyroGenStatus = new PigeonIMU.GeneralStatus();
		ypr_deg = new double[3];
		xyz_dps = new double[3];
		xyz_accl = new short[3];
	}

	/**
	 * Periodically request information from the device
	 */
	public void periodic() {

		pidgey.getFusedHeading(gyrofusionStatus);
		pidgey.getGeneralStatus(gyroGenStatus);
		pidgey.getYawPitchRoll(ypr_deg);
		pidgey.getRawGyro(xyz_dps);
		pidgey.getBiasedAccelerometer(xyz_accl);

		if(pigeonDebug){
			SmartDashboard.putNumber("FusedHeading", pidgey.getFusedHeading());
			SmartDashboard.putNumber("Pitch", getPitch());
			SmartDashboard.putNumber("Roll", getRoll());
		}

		SmartDashboard.putNumber("Pigeon Accl X", q(xyz_accl[0]));
		SmartDashboard.putNumber("Pigeon Accl Y", q(xyz_accl[1]));
		SmartDashboard.putNumber("Pigeon Accl Z", q(xyz_accl[2]));

		SmartDashboard.putNumber("yaw", getYaw());
	}

	private static double q(short val) {
		// Convert Q2.14 -> double
		// http://www.ctr-electronics.com/downloads/api/java/html/classcom_1_1ctre_1_1phoenix_1_1sensors_1_1_pigeon_i_m_u.html#a2a964e72cdd29bd9ca52b24d7d02e326
		// https://en.wikipedia.org/wiki/Q_(number_format)#Q_to_float
		return val * Math.pow(2, -14);
	}

	/**
	 * Gets the angle of the robot on the Z axis (yaw) from the gyro
	 * @return yaw - double angle value of the robot's Z axis
	 */
	public double getYaw() {
		double yaw = 0;
		yaw = ypr_deg[0];
		return yaw;
	}

	public double getYawMod() {
		return getYaw() % 360;
	}

	/**
	 * Returns the pitch from the gyro
	 * @return pitch
	 */
	public double getPitch() {
		double pitch = 0;
		pitch = ypr_deg[1];
		return pitch;
	}

	/**
     * Returns the roll value from the gyro
	 * @return roll
	 */
	public double getRoll() {
		double roll = 0;
		roll = ypr_deg[2];
		return roll;
	}

	/**
	 * Returns the absolute compass heading of the gyro
	 * @return returns the absolute compass heading
	 */
	public double getAbsoluteCompassHeading() {
		return pidgey.getAbsoluteCompassHeading();
	}

	/**
	 * Resets the yaw on the pigeon to 0
	 */
	public void resetPidgey() {
		pidgey.setYaw(0, timeoutMs);
	}

	/**
	 * Gets the rate at which the robot is spinning
	 * @return angularRate
	 */
	public double getAngularRate() {
		double angularRate;
		angularRate = xyz_dps[2];
		return angularRate;
	}

	/**
	 * Use to manually set the yaw in degrees
	 * @param yaw - double value to manually set the yaw (-360 -> 360)
	 */
	public void manualSetYaw(double yaw) {
		yaw *= 64;
		pidgey.setYaw(yaw, timeoutMs);
	}

	 /**
	 * Returns the FusedHeading value from the gyro
	 * @return FusedHeading
	 */
	public double getFusedHeading() {
		return pidgey.getFusedHeading();
	}

	/**
    * Returns the Temperature value from the gyro
	* @return Temp
	*/
   public double getTemp() {
	   return pidgey.getTemp();
   }
}
