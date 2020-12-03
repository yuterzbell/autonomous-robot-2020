package ca.mcgill.ecse211.project;
import static ca.mcgill.ecse211.project.Resources.*;
import static ca.mcgill.ecse211.project.Helper.*;
import static ca.mcgill.ecse211.project.UltrasonicLocalizer.*;
import static simlejos.ExecutionController.sleepFor;
import static ca.mcgill.ecse211.project.LightLocalizer.*;


/**
 * The ObjectDetection class provides method which utilizes the Ultrasonic Sensor to detect potential obstacles and containers 
 * 
 */
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
  public static int topSensorData = readUsDistance2();

  /**
   * Data from Bottom sensor in cm.
   */
  public static int bottomSensorData = readUsDistance();

  @Override
  public void run(){
    Helper.ReinitializeDoubleUsensors();
    while(true){
      bottomSensorData = readUsDistance();
      topSensorData = readUsDistance2();
      sleepFor(500);
    }
  }   


  /**
   * This method will adjust the position of robot to avoid collision.
   * @param bottomSensor
   * @param topSensor
   * @param flag
   */
  public static void objectAvoidance(){
    Helper.turnBy(-90);
    System.out.println("turning");
    int leftSpace = readUsDistance();
    Helper.turnBy(180);
    int rightSpace = readUsDistance();
    if (leftSpace <= rightSpace && rightSpace >= (TILE_SIZE * 100.0)) {
      moveStraightFor(TILE_SIZE);
    } else if (leftSpace > rightSpace && leftSpace >= (TILE_SIZE * 100.0)){
      Helper.turnBy(-180);
      moveStraightFor(TILE_SIZE);
    } else {
      //
    }
  }
  
  /**
   * Detect potential obstacle at close range (within 1 tile)
   * @param int threshold for the down sensor to trigger a response
   * @return boolean true if obstacle detected
   */
  public static boolean obstacleDetect(int downThreshold) {
    ReinitializeDoubleUsensors();
    int down = downMedianFiltering(down_dists);
    int top = topMedianFiltering(top_dists);
//    System.out.println("Top reads: " + top + "\nDown reads: " + down);
    if ((down < downThreshold) && ((top - down) < US_DIFF_THRESHOLD)) {
      OBJ_DIST = down;
      return true;
    }
    return false;
  }
  
}
