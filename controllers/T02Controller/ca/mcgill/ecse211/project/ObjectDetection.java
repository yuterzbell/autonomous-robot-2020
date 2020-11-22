package ca.mcgill.ecse211.project;
import static ca.mcgill.ecse211.project.Resources.*;
import static ca.mcgill.ecse211.project.Helper.*;
import static ca.mcgill.ecse211.project.UltrasonicLocalizer.*;
import static simlejos.ExecutionController.sleepFor;
import static ca.mcgill.ecse211.project.LightLocalizer.*;

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
    while(true){
      detectObject();
      sleepFor(2000);
    }
  }   

  public static void detectObject(){
    topSensorData = readUsDistance2();
//    System.out.println("Top readings: " + topSensorData);
    bottomSensorData = readUsDistance();
//    System.out.println("bottom readings: " + bottomSensorData);
    int sensorDifference = Math.abs(bottomSensorData - topSensorData);
//    System.out.println(DETECT_FLAG);
    if(bottomSensorData < OBJTHRESH && sensorDifference < US_DIFF_THRESHOLD && DETECT_FLAG) {
      DETECT_FLAG = true;
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
   * This method will detect the container by searching a 90-deg sector
   * @return
   */
  public static boolean containerDetect() {
    // turning -180 degrees
    leftMotor.rotate(convertAngle(-180), true);
    rightMotor.rotate(-convertAngle(-180), true);
    ReinitializeDoubleUsensors();
    int down = downMedianFiltering(down_dists);
    int top = topMedianFiltering(top_dists);
    // if found a close enough object.
    while(true) {
      down = downMedianFiltering(down_dists);
//      System.out.println("The down sensor read: " + down);
      top = topMedianFiltering(top_dists);
      if(down < OBJTHRESH) {
        break;
      }
    }
    // if top is really far from down, then it's a container
    if (top > down + US_DIFF_THRESHOLD) {
      System.out.println("Top reads: " + top + "\nDown reads: " + down);
      System.out.println("A container found");
      stop();
      return true;
    }
    return false;
//    moveStraight();
//    while(down > 20) {
//      down = downMedianFiltering(down_dists);
//    }
//    stop();
//    down = downMedianFiltering(down_dists);
//    top = topMedianFiltering(top_dists);
//    if (top > down + US_DIFF_THRESHOLD) {
//      return true;
//    }
//    return false;
  }
  
  public static boolean obstacleDetect() {
    ReinitializeDoubleUsensors();
    int down = downMedianFiltering(down_dists);
    int top = topMedianFiltering(top_dists);
    System.out.println("Top reads: " + top + "\nDown reads: " + down);
    if ((down < OBJTHRESH) && ((top - down) < US_DIFF_THRESHOLD)) {
      OBJ_DIST = down;
      return true;
    }
    return false;
  }
  
//  public static int detectDist() {
//    ReinitializeDoubleUsensors();
//    int down = downMedianFiltering(down_dists);
//    int top = topMedianFiltering(top_dists);
//    System.out.println("Top reads: " + top + "\nDown reads: " + down);
//    if ((down < OBJTHRESH) && ((top - down) < US_DIFF_THRESHOLD)) {
//      System.out.println("Obstackle");
//      return down;
//    }
//    return 0;
//  }
}
