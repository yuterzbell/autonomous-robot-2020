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
  public static int topSensorData = readUsDistance2();

  /**
   * Data from Bottom sensor in cm.
   */
  public static int bottomSensorData = readUsDistance();

  @Override
  public void run(){
    Helper.ReinitializeDoubleUsensors();
    while(true){
//      bottomSensorData = downMedianFiltering(down_dists);
//      topSensorData = topMedianFiltering(top_dists);
      bottomSensorData = readUsDistance();
      topSensorData = readUsDistance2();
      if (bottomSensorData < VALID_OFFSET) {
        objectInClose = true;
         
//        System.out.println("down sensor is in valid distance");
//        System.out.println("Top sensor " + topSensorData);
//        System.out.println("Down sensor " + bottomSensorData);
        
        if (topSensorData > bottomSensorData + US_DIFF_THRESHOLD) {
          isContainer = true;   // TODO not really good design since thread are not synchronized
          sleepFor(500);        // when find a container, delay the update of the next cycle to boolean flags.
        }
      } else {
        objectInClose = false;
        isContainer = false;
      }
      sleepFor(500);
    }
  }   

  /**
   * This method detect a valid object.
   */
  public static void detectObject(){
    topSensorData = readUsDistance2();
//    System.out.println("Top readings: " + topSensorData);
    bottomSensorData = readUsDistance();
//    System.out.println("bottom readings: " + bottomSensorData);
    int sensorDifference = Math.abs(bottomSensorData - topSensorData);
//    System.out.println(DETECT_FLAG);
    if(bottomSensorData < OBJTHRESH && sensorDifference < US_DIFF_THRESHOLD && DETECT_FLAG) {
      DETECT_FLAG = true;
    } else if (bottomSensorData < 10 && sensorDifference > US_DIFF_THRESHOLD && DETECT_FLAG) {
      DETECT_FLAG = true;
      System.out.println("beepo");
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
  }
  
  /**
   * Getter method.
   * @return int botoomSensorData
   */
  public static int getbottomSensorData() {
    return bottomSensorData;
  }
  
  
  public static boolean detect() {
    ReinitializeDoubleUsensors();
    int down = downMedianFiltering(down_dists);
    int top = topMedianFiltering(top_dists);
    System.out.println("Top reads: " + top + "\nDown reads: " + down);
    if ((down < OBJTHRESH) && ((top - down) < US_DIFF_THRESHOLD)) {
      System.out.println("Obstackle");
      return true;
    }
    return false;
  }
  
  public static void avoidObjectInX(boolean up, boolean down) {
    
    double curr_x = odometer.getXyt()[0] / TILE_SIZE;
    double curr_y = odometer.getXyt()[1] / TILE_SIZE;
    double initial_x = curr_x;
    
    boolean rightFlag = false;
    boolean leftFlag = false;
    
    
    if (up) {
      
      Navigation.turnBy(90); //rightCheck
      if (detect()) {
        rightFlag = true;
      }
      
      Navigation.turnBy(-180); //leftCheck
      if (detect()) {
        leftFlag = true;
      }
      
      //restore theta
      Navigation.turnBy(90);
      
      if ((curr_x - szr.ll.x) < 1) { //Check if island border is on left
        
        if (!rightFlag) { //Check if right is clear
          //Try to avoid by moving to right
          cutRightInX(up, down);
        } else {
          //back then cut
        }
      }
    }
    
    if (down) {
      
      Navigation.turnBy(-90); //rightCheck
      if (detect()) {
        rightFlag = true;
      }
      
      Navigation.turnBy(180); //leftCheck
      if (detect()) {
        leftFlag = true;
      }
      
      //restore theta
      Navigation.turnBy(-90);
      
      if ((curr_x - szr.ll.x) < 1) { //Check if island border is on right
        
        if (!leftFlag) { //Check if left is clear
          //Try to avoid by moving to left
          cutLeftInX(up, down);
        } else {
          System.out.println("ERROR");
          //back then cut
        }
      } else if (!leftFlag) {
        cutLeftInX(up, down);
      }
    }
  }
  
  public static void cutLeftInX(boolean up, boolean down) {
    //Note Theta is not updated after each turn as it restores to it intial value
    double curr_x = odometer.getXyt()[0] / TILE_SIZE;
    double curr_y = odometer.getXyt()[1] / TILE_SIZE;
    
    int cutLength = 0;
    
    int factor = 0;
    if (up) {factor = 1;}
    if (down) {factor = -1;}
    

    //temporary solution //to consider if there is obstacle on left
    while (detect()) {
      Navigation.turnBy(-90);
      
      Navigation.moveStraightFor(1);
      curr_x = curr_x + (-factor);
      odometer.setX(curr_x * TILE_SIZE); 
      cutLength++;
      
      Navigation.turnBy(90);
    }
    
    Navigation.moveStraightFor(2);
    curr_y = curr_y + (2*factor);
    odometer.setY(curr_y * TILE_SIZE);
    
    //Restore x and theta
    Navigation.turnBy(90);
    
    Navigation.moveStraightFor(cutLength);
    curr_x = curr_x + (cutLength*factor);
    odometer.setX(curr_x * TILE_SIZE); 
    
    Navigation.turnBy(-90);
   
  }
  
  public static void cutRightInX(boolean up, boolean down) {
    //Note Theta is not updated after each turn as it restores to it intial value
    double curr_x = odometer.getXyt()[0] / TILE_SIZE;
    double curr_y = odometer.getXyt()[1] / TILE_SIZE;
    
    int factor = 0;
    if (up) {factor = 1;}
    if (down) {factor = -1;}
    
    Navigation.turnBy(90);
    
    Navigation.moveStraightFor(1);
    curr_x = curr_x + (factor);
    odometer.setX(curr_x * TILE_SIZE); 
    
    Navigation.turnBy(-90);
    
    if (detect()) {
      avoidObjectInX(up, down);
    } else {
      Navigation.moveStraightFor(2);
      curr_y = curr_y + (2*factor);
      odometer.setY(curr_y * TILE_SIZE);
      
      //Restore x and theta
      Navigation.turnBy(-90);
      
      Navigation.moveStraightFor(1);
      curr_x = curr_x + (-factor);
      odometer.setX(curr_x * TILE_SIZE); 
      
      Navigation.turnBy(90);
    }
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
