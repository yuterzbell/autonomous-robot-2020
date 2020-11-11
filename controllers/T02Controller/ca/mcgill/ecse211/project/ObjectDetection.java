package ca.mcgill.ecse211.project;
import static ca.mcgill.ecse211.project.Resources.*;
import static ca.mcgill.ecse211.project.Helper.*;
import static ca.mcgill.ecse211.project.UltrasonicLocalizer.*;
import static ca.mcgill.ecse211.project.LightLocalizer.*;

public class ObjectDetection implements Runnable{

    /** The singleton odometer instance. */
    private static ObjectDetection odo;

    /**
     * Gets the object for threads
     * @return the object for the thread
     */
    public static synchronized ObjectDetection getObjectDetection() {
        if (odo == null) {
          odo = new ObjectDetection();
        }
        return odo;
      }


    /**
     * Data from Top sensor in cm.
     */
    private static int topSensorData = readUsDistance2();
    
    /**
     * Data from Bottom sensor in cm.
     */
    private static int bottomSensorData = readUsDistance();

    @Override
    public void run(){
        while(true){
            // System.out.println("i am started");
            detectObject(bottomSensorData, topSensorData);
        }
    }   

    public static void detectObject(int bottomSensor, int topSensor){
        int sensorDifference = Math.abs(bottomSensor - topSensor);
        if(sensorDifference <= US_DIFF_THRESHOLD){
            Helper.turnBy(-90);
            moveStraightFor(TILE_SIZE);
            Helper.turnBy(90);
            moveUntilBlackLineDetected2();
        }
    }
}
