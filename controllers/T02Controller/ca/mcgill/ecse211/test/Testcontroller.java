package ca.mcgill.ecse211.test;

import static ca.mcgill.ecse211.project.Resources.*;
import static simlejos.ExecutionController.*;
import java.util.ArrayList;
import ca.mcgill.ecse211.project.LightLocalizer;
import ca.mcgill.ecse211.project.Navigation;
import ca.mcgill.ecse211.project.UltrasonicLocalizer;

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

  /*
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
}