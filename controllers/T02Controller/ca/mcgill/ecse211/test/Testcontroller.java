package ca.mcgill.ecse211.test;

import static ca.mcgill.ecse211.project.Helper.ReinitializeDoubleUsensors;
import static ca.mcgill.ecse211.project.Helper.downMedianFiltering;
import static ca.mcgill.ecse211.project.Helper.topMedianFiltering;
import static ca.mcgill.ecse211.project.Resources.*;
import static simlejos.ExecutionController.*;
import java.util.ArrayList;
import ca.mcgill.ecse211.playingfield.Point;
import ca.mcgill.ecse211.project.LightLocalizer;
import ca.mcgill.ecse211.project.Navigation;
import ca.mcgill.ecse211.project.ObjectDetection;
import ca.mcgill.ecse211.project.UltrasonicLocalizer;

/**
 * This class serves for testing functions of robot.
 */
public class Testcontroller {
  /**
   * This method test the characteristic of USsensor locates at corner after rotating 360 deg.
   * @author Zichen Chang
   */
  public static void USsensorTestatCorner() {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.rotate(UltrasonicLocalizer.convertAngle(360), true);
    rightMotor.rotate(-UltrasonicLocalizer.convertAngle(360), true);
    while(true) {
      System.out.println(UltrasonicLocalizer.readUsDistance());
      sleepFor(500);
    }
  }

  /**
   * This method test the characteristic of USsensortop locates at corner after rotating 360 deg.
   * @author Zichen Chang
   */
  public static void USsensorTestatCorner2() {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.rotate(UltrasonicLocalizer.convertAngle(360), true);
    rightMotor.rotate(-UltrasonicLocalizer.convertAngle(360), true);
    while(true) {
      System.out.println(UltrasonicLocalizer.readUsDistance2());
    }
  }

  /**
   * Moves the robot straight until a black line is detected by both left and right color sensors.
   * Motors will stop once the black line is detected to correct the path. 
   * 
   * @author Andre-Walter Panzini
   */
  public static void moveForOneBlockwithLight() {
    boolean isLeftWheelDetected = false;
    boolean isRightWheelDetected = false;

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    leftMotor.rotate(Navigation.convertDistance(0.3048), true);
    rightMotor.rotate(Navigation.convertDistance(0.3048), true);

    while (!isLeftWheelDetected || !isRightWheelDetected) {
      leftColorSensor.fetchSample(LightLocalizer.getleftColorData(), 0);
      rightColorSensor.fetchSample(LightLocalizer.getrightColorData(), 0);

      System.out.printf("Left: %f\n",LightLocalizer.getleftColorData()[0]);
      System.out.printf("Right: %f\n", LightLocalizer.getrightColorData()[0]); 
    }
  }
  
  /**
   * This controller gathered the readings from double sensors to generate characteristic graph in testing document.
   * @author Zichen Chang
   */
  public static void readDown(){
    leftMotor.setSpeed(MOTOR_LOW);
    rightMotor.setSpeed(MOTOR_LOW);
    leftMotor.rotate(-UltrasonicLocalizer.convertAngle(180), true);
    rightMotor.rotate(UltrasonicLocalizer.convertAngle(180), true);
    while(leftMotor.isMoving()) {
      System.out.println(UltrasonicLocalizer.readUsDistance());
      sleepFor(500);
    }
  }
  
  /**
   * This controller gathered the readings from double sensors to generate characteristic graph in testing document.
   * @author Zichen Chang
   */
  public static void readTop(){
    leftMotor.setSpeed(MOTOR_LOW);
    rightMotor.setSpeed(MOTOR_LOW);
    leftMotor.rotate(-UltrasonicLocalizer.convertAngle(180), true);
    rightMotor.rotate(UltrasonicLocalizer.convertAngle(180), true);
    while(leftMotor.isMoving()) {
      System.out.println(UltrasonicLocalizer.readUsDistance2());
      sleepFor(500);
    }
  }
  
  /**
   * This method tests the containerDetection method.
   * @author Zichen Chang
   */
  public static void testContainerDetection() {
    ReinitializeDoubleUsensors();
    int down = downMedianFiltering(down_dists);
    int top = topMedianFiltering(top_dists);
    System.out.println("Top readings:" + top + "\nDown readings: " + down);
    System.out.println("Is there a container? " + ObjectDetection.containerDetect());
}

  /**
   * This method test the robot's ability to drive up ramp.
   */
  public static void rampUp() {
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    
    double x = 13;
    double y = 7;
    double theta = 0;
    odometer.setXytInTailSize(x, y, theta);
    
    odometer.printPosition();
    Navigation.goRamp();
    
    
  }
  
  /**
   * This method test the ability of robot to detect water.
   */
  public static void waterDetect() {
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    double x = 8;
    double y = 7;
    double theta = 180;
    
    double waterX = 8;
    double waterY = 4;
    
    Point water = new Point(waterX, waterY);
    
    odometer.setXytInTailSize(x, y, theta);
//    Navigation.travelTo(water);
    
  }
  
  /**
   * This method test the ability of robot to avoid obstacle.
   */
  public static void obstacleDetection() {
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    double x = 1;
    double y = 8;
    double theta = 90;
    
    double finalX = 3;
    double finalY = 7.5;
    
     
    odometer.setXytInTailSize(x, y, theta);
    Point finalD = new Point(finalX, finalY);
//    Navigation.travelTo(finalD);
    
  }
  
  /**
   * This method test the ability of the robot to simply go up the ramp. Hardware test
   */
  public static void rampTestHardware() {
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    
    Navigation.forward();
  
  }
  
  /**
   * This method test the ability of the robot to avoid obstacles.
   */
  public static void obstacleAvoid() {
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    double x = 1;
    double y = 8;
    double theta = 90;
    
    double xDest = 3.5;
    double yDest = 7.5;
    
    Point finalD = new Point(xDest, yDest);
    odometer.setXytInTailSize(x, y, theta);
    Navigation.navigateTo(finalD);
  
    
  }
  
  /**
   * This method test the ability to get container mass.
   */
  public static void getMass() {
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    
    STORE_TORQUE = true;
    Navigation.moveStraightFor(2.0);
    STORE_TORQUE = false;
    Navigation.calculateMass();
    
  }
}