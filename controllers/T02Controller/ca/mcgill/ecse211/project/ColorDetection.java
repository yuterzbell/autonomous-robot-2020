package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import simlejos.robotics.SampleProvider;


/**
 * @author Andre-Walter Panzini
 * 
 * This class is responsible for detecting the water
 * Also anything related to using the color sensor values
 */
public class ColorDetection {

  /** Color sensors are in RGB mode */
  private static SampleProvider leftColorSensorSample = leftColorSensor.getRGBMode();
  private static SampleProvider rightColorSensorSample = rightColorSensor.getRGBMode();
  
  /** Buffer (array) to store US samples. */
  private static float[] leftColorSensorData = new float[leftColorSensorSample.sampleSize()];
  private static float[] rightColorSensorData = new float[rightColorSensorSample.sampleSize()];
  
  /** Threshold to determine if euclidean distance is reasonable to conclude for the color */
  private static int eucDisThresh = 500;
  
  /** Additional distance to move to detect water after line (in feet)*/
  private static float distAfterBlackLine = (float) 0.02;
  
  /**
   * reads the value of the light sensor into the data variable 
   */
  public static void getLightSensorReadings() {
    leftColorSensorSample.fetchSample(leftColorSensorData, 0);
    rightColorSensorSample.fetchSample(rightColorSensorData, 0);
  }
  
  /**
   * @author Andre-Walter Panzini
   * 
   * Checks to see if the light sensor detects a blue zone
   * A blue zone in this case is water
   * @return isBlueZone; true - if blue zone is detected, else false
   */
  public static boolean getIfBlueZone() {
    boolean isBlueZone = false;
    String color;
    
    getLightSensorReadings();
    
    color = getColorFromSensor(leftColorSensorData);
    
    isBlueZone = color.equals(COLOR_ARR[3] /*BLUE*/);
    
    if(!isBlueZone) {
      color = getColorFromSensor(rightColorSensorData);
      isBlueZone = color.equals(COLOR_ARR[3] /*BLUE*/);
    }
      
    return isBlueZone;
  }
  
  /**
   * @author Andre-Walter Panzini
   * 
   * Gets the color with the least euclidean distance from the sensor
   * @param colorSensorData the light sensor color data
   * @return a color with the smallest euclidean distance or BLACK if the smallest color is greater than the threshold
   */
  public static String getColorFromSensor(float[] colorSensorData) {
    float[] eucledianDistanceArr = new float[4];
    int indexOfSmallest = 0; 
    for(int i = 0; i < 4 ; i++) {
      eucledianDistanceArr[i] = computeRGBEuclideanDistance(colorSensorData, COLOR_ARR[i]);
    }
    for(int i = 0 ; i < 3 ; i++) {
      if(eucledianDistanceArr[indexOfSmallest] > eucledianDistanceArr[i+1]) {
        indexOfSmallest = i + 1;
      }
    }
    
    System.out.println("EucDis: Red: " + eucledianDistanceArr[0] + " Green: " + eucledianDistanceArr[1] + " Yellow: " + eucledianDistanceArr[2] + " Blue: " + eucledianDistanceArr[3]);
    
    if( eucledianDistanceArr[indexOfSmallest] > eucDisThresh) {
      return "UNCERTAIN";
    }
    
    return COLOR_ARR[indexOfSmallest];
  }
  
  // Pass a color from the COLOR_ARR (defined in Resources)
  // We implement integer algebra from float values
  public static float computeRGBEuclideanDistance(float[] colorSensorData, String color) {
    float euclideanDistance;
    int tmpsquare = 0;
    float square = 0;
    int r = (int)(colorSensorData[0] * 100);
    int g = (int)(colorSensorData[1] * 100);
    int b = (int)(colorSensorData[2] * 100);
    int r_model; 
    int g_model; 
    int b_model; 
    
    switch(color) {
      case "RED": {
        r_model = (int) (R_mean_RED * 100);
        g_model = (int) (G_mean_RED * 100);
        b_model = (int) (B_mean_RED * 100);
        tmpsquare = (r_model - r) * (r_model - r) + (g_model - g) * (g_model - g) + (b_model - b) * (b_model - b);
      }
      break;
      case "GREEN": {
        r_model = (int) (R_mean_GREEN * 100);
        g_model = (int) (G_mean_GREEN * 100);
        b_model = (int) (B_mean_GREEN * 100);
        tmpsquare = (r_model - r) * (r_model - r) + (g_model - g) * (g_model - g) + (b_model - b) * (b_model - b);
      }
      break;
      case "YELLOW": {
        r_model = (int) (R_mean_YELLOW * 100);
        g_model = (int) (G_mean_YELLOW * 100);
        b_model = (int) (B_mean_YELLOW * 100);
        tmpsquare = (r_model - r) * (r_model - r) + (g_model - g) * (g_model - g) + (b_model - b) * (b_model - b);
      }
      break;
      case "BLUE": {
        r_model = (int) (R_mean_BLUE * 100);
        g_model = (int) (G_mean_BLUE * 100);
        b_model = (int) (B_mean_BLUE * 100);
        tmpsquare = (r_model - r) * (r_model - r) + (g_model - g) * (g_model - g) + (b_model - b) * (b_model - b);
      }
      break;
    }
    square = (float) tmpsquare/(float) 100.0;
    euclideanDistance = (float) Math.sqrt((double) square);
    return euclideanDistance;
  }
  
  public static void moveUntilWaterDetected() {
    boolean isWaterDetected = false;
   
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
       
    while (!isWaterDetected) {
      rightMotor.setSpeed(ROTATE_SPEED);
      leftMotor.setSpeed(ROTATE_SPEED);
      
      isWaterDetected = getIfBlueZone();
      
      if(isWaterDetected) {
        leftMotor.stop();
        rightMotor.stop();
        Navigation.moveStraightFor(-0.2);
        Navigation.turnBy(180);
        System.out.println("WATER DETECTED STOP!");
      } else {
        leftMotor.forward();
        rightMotor.forward();
      } 
    }
  }
  
  /**
   * Moves the robot for a certain distance and corrects the orientation of the robot
   * when it crosses a black line.
   * 
   * @param moveInX integer indicating robot is move in x-direction if equal to 1
   * @param distance number of tiles (in feet) to move straight for
   */
  public static boolean moveStraightWithLineCorrectionAndWaterDetection(int moveInX, double distance) { 
    double inMeters = distance * TILE_SIZE;
    double distanceChange = 0;
    boolean waterDetected = false;

    while (inMeters > TILE_SIZE) {    // while distance left can cover another tile
      if (DETECT_FLAG) {
        ObjectDetection.objectAvoidance();
      }
      var xyt0 = odometer.getXyt();
      
      LightLocalizer.moveUntilBlackLineDetected();
      Helper.moveStraightFor(distAfterBlackLine);
      waterDetected = getIfBlueZone();
      if(waterDetected) {
        leftMotor.stop();
        rightMotor.stop();
        System.out.println("WATER DETECTED");
        Navigation.moveStraightFor(-0.2);
        Navigation.turnBy(180);
        break;
        //TODO what is the expected behavior if water is detected?
      }
      Helper.moveStraightFor(COLOR_SENSOR_TO_WHEEL_DIST - distAfterBlackLine);        // TODO remember to change the speed here to FORWARD_SPEED
     
      var xyt1 = odometer.getXyt();
      
      if (moveInX == 1) {
        double x = xyt1[0] / TILE_SIZE;             // calculate the x-coordinate
        odometer.setX(Math.round(x) * TILE_SIZE);   // round the x-coordinate
        if (xyt1[2] < 100 && xyt1[1] > 80) {
          odometer.setTheta(90);
        } else if (xyt1[2] < 280 && xyt1[1] > 260){
          odometer.setTheta(270);
        }
      } else if (moveInX == 0){
        double y = xyt1[1] / TILE_SIZE;             // calculate the x-coordinate
        odometer.setY(Math.round(y) * TILE_SIZE);   // round the x-coordinate
        if (xyt1[2] < 10 && xyt1[1] > 350) {
          odometer.setTheta(0);
        } else if (xyt1[2] < 190 && xyt1[1] > 170){
          odometer.setTheta(180);
        }
      }
      distanceChange = Math.sqrt(Math.pow((xyt0[1] - xyt1[1]), 2) + Math.pow((xyt0[0] - xyt1[0]), 2));
      inMeters -= distanceChange;
    }
    
    if(!waterDetected) {
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
      Navigation.moveStraightFor(inMeters / TILE_SIZE);
    }
    
    return !waterDetected;
   }

}