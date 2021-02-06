package frc.robot.subsystems;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import edu.wpi.first.hal.can.CANExceptionFactory;
import edu.wpi.first.hal.can.CANJNI;

/**
 * Receives and sends information to the device
 */
public class CANSendReceive {

    public static ByteBuffer targetID = ByteBuffer.allocateDirect(4);
    private static ByteBuffer timeStamp = ByteBuffer.allocateDirect(4);
    /** An array of bytes that is set to  */
    public static byte[] result;
    public static int status;


    /**
     * Helper routine to get last received message for a given ID.
     * @param fullId The message ID being read.
     * @param deviceID The ID of the device
     * @return Returns the timestamp as an integer. If it returns -1, it failed.
     */
    public static long readMessage(int fullId, int deviceID) {
        //Keep the try/catch loop so that we can handle the exception
        try {
            // This code was copied from 
            // https://www.programcreek.com/java-api-examples/index.php?api=edu.wpi.first.wpilibj.can.CANJNI
            targetID.clear();
            targetID.order(ByteOrder.LITTLE_ENDIAN);
            targetID.asIntBuffer().put(0, fullId | deviceID);

            timeStamp.clear();
            timeStamp.order(ByteOrder.LITTLE_ENDIAN);
            timeStamp.asIntBuffer().put(0, 0x00000000);

            //Receives a message from the CANbus and assigns it to an integer buffer
            result = CANJNI.FRCNetCommCANSessionMuxReceiveMessage(targetID.asIntBuffer(), 0x1fffffff, timeStamp);

            long retval = timeStamp.getInt();
            retval &= 0xFFFFFFFF; /* undo sign-extension */
            return retval;

        } catch (Exception e) {

            //System.out.println(e);
            return -1;
        }
    }

    /**
     * Sends a message to the sensor
     * @param messageID The message ID
     * @param data The data being sent
     * @param dataSize The size of the data
     * @param period The send message period (kSendMessagePeriod)
     */
    public static void sendMessage(int messageID, byte[] data, int dataSize, int period) {

        CANJNI.FRCNetCommCANSessionMuxSendMessage(messageID, data, period);
        CANExceptionFactory.checkStatus(status, messageID);

    }

}
