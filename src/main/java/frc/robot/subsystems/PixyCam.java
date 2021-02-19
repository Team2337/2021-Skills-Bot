package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import io.github.pseudoresonance.pixy2api.*;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import frc.robot.Constants.PixyAutons;

/**
 * The code for retrieving PixyCam values from the SPI
 * @author Michael F.
 */
public class PixyCam extends SubsystemBase {

	//PixyCam
	private Pixy2 pixycam;
	private boolean isCamera = false;
	private int state = -1;
	private int chip;
	private int numberOfTargets = 0;
	private boolean seesTarget = false;

	//For efficiency
	private int cacheNumber = 0;
	private int lastLargestBlockRetrieval = -1;
	private Block lastLargestBlock;

	//Debug mode
	private final boolean DEBUG = true;

	/**
	 * Subsystem for the PixyCam
	 * @param chipselect The chip the pixy is plugged into on the SPI
	 */
	public PixyCam(int chipselect){
		chip = chipselect;
		pixycam = Pixy2.createInstance(Pixy2.LinkType.SPI);
		state = pixycam.init(chip);
	}

	@Override
	public void periodic(){
		// This method will be called once per scheduler run

		//If there is no camera present, try to initialize it.
		if(!isCamera)
			state = pixycam.init(chip);

		//Detect connection
		isCamera = (state >= 0);
		SmartDashboard.putNumber("Pixy " + chip + " State", state);
		SmartDashboard.putBoolean("Pixy " + chip + " Connected", isCamera);
		SmartDashboard.putBoolean("Pixy " + chip + " sees target", seesTarget);

		if(DEBUG){
			//Acquire target data
			updateTargets();
			if(numberOfTargets > 0){
				//Get the largest target
				// Block lt = getLargestTarget(); //Gets the largest target (lt)
				SmartDashboard.putString("Largest block", getLargestTarget().toString());
				SmartDashboard.putNumber("Largest Target X-Coord", getLargestTargetX());
				SmartDashboard.putNumber("Largest Target Y-Coord", getLargestTargetY());
				SmartDashboard.putNumber("Largest Target Angle", getLargestTargetAngle());
				SmartDashboard.putNumber("Largest Target Width", getLargestTargetWidth());
				SmartDashboard.putNumber("Largest Target Height", getLargestTargetHeight());
			}
			//Push to dashboard how many targets are detected
			SmartDashboard.putNumber("Number of Targets", numberOfTargets); 
		}
	}

	/**
	 * Refreshes the target cache.
	 */
	public void updateTargets(){
		//Retrieve the targets and store the number in a variable
		numberOfTargets = pixycam.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 48);
		//Update the cache number
		cacheNumber++;
		//Update the seesTarget variable
		if(numberOfTargets > 0) seesTarget = true;
		else seesTarget = false;
	}

	/**
	 * Gets all cached targets. Be sure to update it with updateTargets()
	 * @return An ArrayList of target data.
	 */
	public ArrayList<Block> getAllTargets(){
		//Retrieve all blocks
		ArrayList<Block> blocks = pixycam.getCCC().getBlockCache();
		return blocks;
	}

	/**
	 * Gets the largest target.
	 * @return A Block class containing the largest target.
	 * @see Block
	 */
	public Block getLargestTarget(){
		//See if we already have the largest Block (to be efficient)
		if(lastLargestBlockRetrieval == cacheNumber){
			SmartDashboard.putNumber("lastRetrieval", lastLargestBlockRetrieval);
			SmartDashboard.putNumber("cacheNumber", cacheNumber);
			return lastLargestBlock;
		}

		//Check to see if there are any targets.
		if(numberOfTargets <= 0){
			return null;
		}
		//Get all the targets
		ArrayList<Block> blocks = getAllTargets();
		Block largestBlock = null;
		//Loops through all targets and finds the widest one
		for(Block block : blocks){
			if(largestBlock == null){
				//If this is the first iteration, set largestBlock to the current block.
				largestBlock = block;
			} else if(block.getWidth() > largestBlock.getWidth()){
				//If we find a wider block, set largestBlock to the current block.
				largestBlock = block;
			}
		}

		//Update the last time we looked for the largest Block
		lastLargestBlockRetrieval = cacheNumber;
		//Store this Block
		lastLargestBlock = largestBlock;
		//Return the Blocks
		SmartDashboard.putString("Largest block", largestBlock.toString());
		return largestBlock;
	}

	/**
	 * @return Returns the x-coordinate of the largest target. 0-315
	 */
	public int getLargestTargetX(){
		Block largestTarget = getLargestTarget();
		if(largestTarget == null) return -1;
		return largestTarget.getX();
	}
	/**
	 * @return Returns the y-coordinate of the largest target. 0-207
	 */
	public int getLargestTargetY(){
		Block largestTarget = getLargestTarget();
		if(largestTarget == null) return -1;
		return largestTarget.getY();
	}
	/**
	 * @return Returns the angle to the largest target in degrees from the center of the camera.
	 * Ranges from -30 to 30. Returns 0.0 if no target was found.
	 */
	public double getLargestTargetAngle(){
		double x = (double)getLargestTargetX();
		if(x == -1) return 0.0;
		//316 is the width of the camera
		return ((x / 316) * 60) - 30;
	}
	/**
	 * @return Returns the width of the largest target.
	 */
	public int getLargestTargetWidth(){
		Block largestTarget = getLargestTarget();
		if(largestTarget == null) return -1;
		return largestTarget.getWidth();
	}
	/**
	 * @return Returns the height of the largest target.
	 * This is generally going to be smaller than the width because of lighting.
	 */
	public int getLargestTargetHeight(){
		Block largestTarget = getLargestTarget();
		if(largestTarget == null) return -1;
		return largestTarget.getHeight();
	}

	/**
	 * Detects which path should be used
	 * @return An enum representing the path to take. Returns null if there was an error.
	 */
	public PixyAutons getAuton(){
		updateTargets();
		int x = getLargestTargetX();
		if(x >= 0 && x < 79)          return PixyAutons.RedA;
		else if(x >= 79 && x < 158)   return PixyAutons.RedB;
		else if(x >= 158 && x < 237)  return PixyAutons.BlueA;
		else if(x >= 237 && x <= 315) return PixyAutons.BlueB;
		else return null;
	}
}
