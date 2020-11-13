package ca.mcgill.ecse211.project;
import static ca.mcgill.ecse211.project.Resources.*;
import static ca.mcgill.ecse211.project.Helper.*;
import static ca.mcgill.ecse211.project.UltrasonicLocalizer.*;
import static ca.mcgill.ecse211.project.LightLocalizer.*;
import static simlejos.ExecutionController.waitUntilNextStep;

public class ObjectDetection implements Runnable{

    /** The singleton odometer instance. */
    private static ObjectDetection detect;

    /**
     * Gets the object for threads
     * @return the object for the thread
     */
    public static synchronized ObjectDetection getObjectDetection() {
        if (detect == null) {
          detect = new ObjectDetection();
        }
        return detect;
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
        while(!boxDetected){
            // System.out.println("i am started");
            topSensorData = readUsDistance2();
            bottomSensorData = readUsDistance();
            detectObject(bottomSensorData, topSensorData);
            waitUntilNextStep();
        }
    }   

    public static void detectObject(int bottomSensor, int topSensor){
        int sensorDifference = Math.abs(bottomSensor - topSensor);
        if (bottomSensor < 25) { // to be calibrated to an appropriate threshold //
            leftMotor.stop();
            rightMotor.stop();
            goCheck = true;
            if (sensorDifference < 20) {
                System.out.println("Obstacle");
                clear = false;
            } else {
                System.out.println("Box");
                boxDetected = true;
            }
        }
    }
}
