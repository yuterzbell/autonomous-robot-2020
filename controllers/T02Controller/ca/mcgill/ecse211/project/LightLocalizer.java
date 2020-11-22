package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import java.util.Arrays;

/**
 * Uses the lightsensor to perform light localization.
 */
public class LightLocalizer {
 
  /** Buffer (array) to store US samples. */
  private static float[] leftColorSensorData = new float[leftColorSensor.sampleSize()];
  private static float[] rightColorSensorData = new float[rightColorSensor.sampleSize()];
  
  /** The discrete derivatives of each sensor. */
  private static int[] leftDerivative = new int[3];
  private static int[] rightDerivative = new int[3];
 
  /** Values read from corresponding light sensor. */  
  private static int[] leftValues = new int[5];
  private static int[] rightValues = new int[5];
 
  /** Counters */
  private static int i = 0;     // left counter
  private static int j = 0;     // right counter
  
  /** Last readings from sensor. */
  private static int prevLeft;
  private static int prevRight;
  
  /** Derivative threshold for valid change in readings. */
  private static int dThresh = 50;
  
  /**
   * This method will bring the pivot point of the robot a the top right corner 
   * of the bottom left tile of the world. Point (1,1)
   *
   */
  public static void localize() {
    
    // The robot must move forward until both sensors detect a dark line
    moveUntilBlackLineDetected();
    
    // The robot must move back the distance between the wheels and the sensors
    Helper.moveStraightFor(COLOR_SENSOR_TO_WHEEL_DIST);
    
    // The robot must turn 90 degrees clockwise
    Helper.turnBy(90);
    
    // The robot must move forward until both sensors detect a dark line
    moveUntilBlackLineDetected();

    // The robot must move back the distance between the wheels and the sensors
    Helper.moveStraightFor(COLOR_SENSOR_TO_WHEEL_DIST);

    // The robot must turn 90 degrees anticlockwise and should 
    Helper.turnBy(-90);
  }

 
  /*
   * Moves the robot straight until a black line is detected by both left and right color sensors.
   * Motors will stop once the black line is detected to correct the path. 
   * 
   * @author Andre-Walter Panzini
   */
/*
  public static void moveUntilBlackLineDetected() {
    boolean isLeftWheelDetected = false;
    boolean isRightWheelDetected = false;
    
    leftMotor.setSpeed(MOTOR_LOW);
    rightMotor.setSpeed(MOTOR_LOW);
    
    while (!isLeftWheelDetected || !isRightWheelDetected) {
      leftColorSensor.fetchSample(leftColorSensorData, 0);
      rightColorSensor.fetchSample(rightColorSensorData, 0);
     
//      System.out.printf("Left: %f\n",leftColorSensorData[0]);
//      System.out.printf("Right: %f\n", rightColorSensorData[0]);
      
      if (leftColorSensorData[0] < LIGHT_THRESH) { //Should update the threshold to be a constant
        leftMotor.stop();
        isLeftWheelDetected = true;
      }
      else {
        leftMotor.forward();
      }
        
      if (rightColorSensorData[0] < LIGHT_THRESH) {
        rightMotor.stop();
        isRightWheelDetected = true;
      }
      else {
        rightMotor.forward();
      }
      
    }
  } 
  */
  
  /**
   * Moves the robot straight until a black line is detected by both left and right color sensors.
   * Motors will stop once the black line is detected to correct the path. 
   * This method using Zero-crossing concept to detect black line.
   * 
   * @author Andre-Walter Panzini and Zichen Chang
   */
  public static void moveUntilBlackLineDetected() {
    boolean isLeftWheelDetected = false;
    boolean isRightWheelDetected = false;
   
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    
    initializeData();
   
    while (!isLeftWheelDetected || !isRightWheelDetected) {
      if (isLeftWheelDetected) {
        rightMotor.setSpeed(ROTATE_SPEED);
      } 
      if (isRightWheelDetected) {
        leftMotor.setSpeed(ROTATE_SPEED);
      }
      // get the filtered readings
      int left = medianFilteringLeft(leftValues);      
      int right = medianFilteringRight(rightValues);
      
      // then record derivative of the 
      int derivative = 0;
      derivative = left - prevLeft;
      if (Math.abs(derivative) > dThresh && i < 2) {
        leftDerivative[i] = derivative;
        i++;
      }
      prevLeft = left;
      
      derivative = right - prevRight;
      if (Math.abs(derivative) > dThresh && j < 2) {
        rightDerivative[j] = derivative;
        j++;
      }
      prevRight = right;
      
      
//      System.out.printf("Left: %d\n",left);
//      System.out.printf("Right: %d\n", right);
      
      if (leftDerivativeValid()) { // Should update the threshold to be a constant
        leftMotor.stop();
        isLeftWheelDetected = true;
        clearLeftDerivatives();
      } else {
        leftMotor.forward();
      }
        
      if (rightDerivativeValid()) {
        rightMotor.stop();
        isRightWheelDetected = true;
        clearRightDerivatives();
      } else {
        rightMotor.forward();
      }
      
    }
  }
  
  /**
   * THis method initialize the window of two data readings from sensors.
   * @author Zichen Chang
   */
  private static void initializeData() {
    // initialize the window of our data
    int i = 0;                // counter of initializing dist[]
    while (i < leftValues.length) {
      leftColorSensor.fetchSample(leftColorSensorData, 0);
      leftValues[i] = (int) leftColorSensorData[0];
      rightValues[i] = (int) leftColorSensorData[0];
      i++;
    }
    prevLeft = (int) leftColorSensorData[0];
    prevRight = (int) leftColorSensorData[0];
  }
  
  
  /**
   * Returns true when two sharp derivatives have been detected by left sensor.
   * @return true when two sharp derivatives have been detected by left sensor.
   * @author Zichen Chang
   */
  public static boolean leftDerivativeValid() {
    if (Math.abs(leftDerivative[0]) > dThresh &&
        Math.abs(leftDerivative[1]) > dThresh) {
      return true;
    }
    return false;
  }
  
  /**
   * Returns true when two sharp derivatives have been detected by right sensor.
   * @return true when two sharp derivatives have been detected by right sensor.
   * @author Zichen Chang
   */
  public static boolean rightDerivativeValid() {
    if (Math.abs(rightDerivative[0]) > dThresh &&
        Math.abs(rightDerivative[1]) > dThresh) {
      return true;
    }
    return false;
  }
  
  /**
   * This method clears the derivative arrays and counter.
   * @author Zichen Chang
   */
  public static void clearLeftDerivatives() {
    i = 0;
    leftDerivative = new int[3];
  }
  
  /**
   * This method clears the derivative arrays and counter.
   * @author Zichen Chang
   */
  public static void clearRightDerivatives() {
    j = 0;
    rightDerivative = new int[3];
  }
  
  /**
   * Get the readings from left LightSensor
   * @return array of left sensor readings
   */
  public static float[] getleftColorData() {
    return leftColorSensorData;
  }
  
  /**
   * Get the readings from right LightSensor
   * @return array of right sensor readings
   */
  public static float[] getrightColorData() {
    return rightColorSensorData;
  }
  
  /**
   * Use median window filtering to filter data.
   * @return  median of the window
   * @author Zichen Chang
   */
  private static int medianFilteringLeft(int[] arr) {
    // shift data window to left by 1
    for (int j = 0; j < arr.length - 1; j++) {
      arr[j] = arr[j + 1];
    }
    leftColorSensor.fetchSample(leftColorSensorData, 0);      // get distance of current loop
    int value = (int)leftColorSensorData[0];
    arr[arr.length - 1] = value;

    // return the filtered data
    return getMedian(arr);
  }
  
  /**
   * Use median window filtering to filter data.
   * @return  median of the window
   * @author Zichen Chang
   */
  private static int medianFilteringRight(int[] arr) {
    // shift data window to left by 1
    for (int j = 0; j < arr.length - 1; j++) {
      arr[j] = arr[j + 1];
    }
    rightColorSensor.fetchSample(rightColorSensorData, 0);      // get distance of current loop
    int value = (int)rightColorSensorData[0];
    arr[arr.length - 1] = value;

    // return the filtered data
    return getMedian(arr);
  }
  
  /**
   * Returns median of a given array.
   * @param arr input array
   * @return the filtered median data of the window
   */
  public static int getMedian(int[] arr) {
    // System.out.println(arr[0] + ", " + arr[1] + ", " + arr[2] + ", " + arr[3] + ", " + arr[4]);
    int[] copy = arr.clone();
    Arrays.sort(copy);
    int median = 0;
    if (copy.length % 2 == 0) {
      median = (copy[copy.length / 2] + copy[copy.length / 2 - 1]) / 2;
    } else {
      median = copy[copy.length / 2];
    }
    return median;
  }
   
}
