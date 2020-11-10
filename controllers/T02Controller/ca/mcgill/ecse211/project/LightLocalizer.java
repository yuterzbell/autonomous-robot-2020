package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;


public class LightLocalizer {
 
  /** Buffer (array) to store US samples. */
  private static float[] leftColorSensorData = new float[leftColorSensor.sampleSize()];
  private static float[] rightColorSensorData = new float[rightColorSensor.sampleSize()];
  /** The discrete derivatives of each sensor. */
  private static float[] leftDerivative = new float[3];
  private static float[] rightDerivative = new float[3];
  private static int i = 0;     // left counter
  private static int j = 0;     // right counter
  /** Last readings from sensor. */
  private static float prevLeft;
  private static float prevRight;
  /** Derivative threshold for valid change in readings. */
  private static float dThresh = 50f;
  
  /**
   * This method will bring the pivot point of the robot a the top right corner 
   * of the bottom left tile of the world. Point (1,1)
   *
   */
  public static void localize() {
    leftColorSensor.fetchSample(leftColorSensorData, 0);
    rightColorSensor.fetchSample(rightColorSensorData, 0);
    prevLeft = leftColorSensorData[0];
    prevRight = rightColorSensorData[0];
    
    // The robot must move forward until both sensors detect a dark line
    moveUntilBlackLineDetected2();
    
    // The robot must move back the distance between the wheels and the sensors
    Helper.moveStraightFor(COLOR_SENSOR_TO_WHEEL_DIST);
    
    // The robot must turn 90 degrees clockwise
    Helper.turnBy(90);
    
    // The robot must move forward until both sensors detect a dark line
    moveUntilBlackLineDetected2();

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
    
    leftMotor.setSpeed(MOTOR_HIGH);
    rightMotor.setSpeed(MOTOR_HIGH);
    
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
  } */
  
  /*
   * Moves the robot straight until a black line is detected by both left and right color sensors.
   * Motors will stop once the black line is detected to correct the path. 
   * This method using Zero-crossing concept to detect black line.
   * 
   * @author Andre-Walter Panzini and Zichen Chang
   */
  public static void moveUntilBlackLineDetected2() {
    boolean isLeftWheelDetected = false;
    boolean isRightWheelDetected = false;
    
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    
    while (!isLeftWheelDetected || !isRightWheelDetected) {
      readLight();
//      System.out.printf("Left: %f\n",leftColorSensorData[0]);
//      System.out.printf("Right: %f\n", rightColorSensorData[0]);
      
      if (leftDerivativeValid()) { //Should update the threshold to be a constant
        leftMotor.stop();
        isLeftWheelDetected = true;
        System.out.println("left black line detected");
        clearLeftDerivatives();
      } else {
        leftMotor.forward();
      }
        
      if (rightDerivativeValid()) {
        rightMotor.stop();
        isRightWheelDetected = true;
        System.out.println("right black line detected");
        clearRightDerivatives();
      } else {
        rightMotor.forward();
      }
      
    }
  }
  
  
  /**
   * This method read the values from light sensors and update derivative arrays.
   * @author Zichen Chang
   */
  public static void readLight() {
    float derivative = 0;
    leftColorSensor.fetchSample(leftColorSensorData, 0);
    float currentLeft = leftColorSensorData[0];
    derivative = currentLeft - prevLeft;
    if (Math.abs(derivative) > dThresh && i < 2) {
      leftDerivative[i] = derivative;
      i++;
    }
    prevLeft = currentLeft;
    
    rightColorSensor.fetchSample(rightColorSensorData, 0);
    float currentRight = rightColorSensorData[0];
    derivative = currentRight - prevRight;
    if (Math.abs(derivative) > dThresh && j < 2) {
      rightDerivative[j] = derivative;
      j++;
    }
    prevRight = currentRight;
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
   */
  public static void clearLeftDerivatives() {
    i = 0;
    leftDerivative = new float[3];
  }
  
  /**
   * This method clears the derivative arrays and counter.
   */
  public static void clearRightDerivatives() {
    j = 0;
    rightDerivative = new float[3];
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
   
}
