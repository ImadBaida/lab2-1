package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;

/**
 * This class is used to drive the robot on the demo floor.
 */
public class SquareDriver {

  /**
   * Drives the robot in a square of the given length. It is to be run in parallel
   * with the odometer to allow testing its functionality.
   *
   * @param length the length of the square in feet (tile sizes)
   */
  public static void driveInASquare(double length) {
    setAcceleration(ACCELERATION);
    for (int i = 0; i < 4; i++) {
      moveStraightFor(length);
      turnBy(90.0); // degrees clockwise
    }
    stopMotors();
  }
  
  /**
   * Moves the robot straight for the given distance.
   *
   * @param distance in feet (tile sizes), may be negative
   */
  public static void moveStraightFor(double distance) {
    // Sets motor speeds and rotate them by the given distance.
    // Uses blocking argument to not return until the robot has finished moving.
    setSpeed(FORWARD_SPEED);
    //calculate angle to rotate. Multiply by TILE_SIZE to get distance in metres. 
    int rotAngle = convertDistance(distance * TILE_SIZE);  
    //return set to true, so that rightMotor starts rotating at the same time
    leftMotor.rotate(rotAngle, true);   
    //return set to false, so that the method does not return until rotation finished
    rightMotor.rotate(rotAngle, false); 
    //stops motors
    stopMotors();

  }
  
  /**
   * Turns the robot by a specified angle. Note that this method is different from
   * {@code Navigation.turnTo()}. For example, if the robot is facing 90 degrees, calling
   * {@code turnBy(90)} will make the robot turn to 180 degrees, but calling
   * {@code Navigation.turnTo(90)} should do nothing (since the robot is already at 90 degrees).
   *
   * @param angle the angle by which to turn, in degrees
   */
  public static void turnBy(double angle) {
    // Sets motor speeds and rotate them by the given distance.
    // Uses blocking argument to not return until the robot has finished moving.
    setSpeed(FORWARD_SPEED);
    //calculate angle to rotate. 
    int rotAngle = convertAngle(angle);  
    //return set to true, so that rightMotor starts rotating at the same time
    leftMotor.rotate(rotAngle, true);   
    //return set to false, so that the method does not return until rotation finished. 
    //uses negative for a clockwise turn.
    rightMotor.rotate(-rotAngle, false); 
    //stops motors
    stopMotors();
    
  }
  
  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   *
   * @param distance the input distance in meters
   * @return the wheel rotations necessary to cover the distance in degrees
   */
  public static int convertDistance(double distance) {
    //calculates rotation in degrees based on distance and wheel radius. 
    //No need to make conversions since both values are given in metres.
    return (int) (distance * 180 / (Math.PI * WHEEL_RAD));
    
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that
   * angle.
   *
   * @param angle the input angle in degrees
   * @return the wheel rotations necessary to rotate the robot by the angle in degrees
   */
  public static int convertAngle(double angle) {
    // Computes and returns the correct value. Hint: you can reuse convertDistance()
    return (int) (angle * BASE_WIDTH / WHEEL_RAD);
  }
  
  /**
   * Stops both motors.
   */
  public static void stopMotors() {
    leftMotor.stop();
    rightMotor.stop();
  }
  
  /**
   * Sets the speed of both motors to the same values.
   *
   * @param speed the speed in degrees per second
   */
  public static void setSpeed(int speed) {
    // Reuses existing method to set both speeds to same value
    setSpeeds(speed, speed); 
  }
  
  /**
   * Sets the speed of both motors to different values.
   *
   * @param leftSpeed the speed of the left motor in degrees per second
   * @param rightSpeed the speed of the right motor in degrees per second
   */
  public static void setSpeeds(int leftSpeed, int rightSpeed) {
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);
  }
  
  /**
   * Sets the acceleration of both motors.
   *
   * @param acceleration the acceleration in degrees per second squared
   */
  public static void setAcceleration(int acceleration) {
    leftMotor.setAcceleration(acceleration);
    rightMotor.setAcceleration(acceleration);
  }

}
