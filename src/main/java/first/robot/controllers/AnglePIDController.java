package first.robot.controllers;

import org.wpilib.math.controller.PIDController;

/**
 * PID controller specialized for angles in degrees.
 *
 * <p>This controller automatically wraps angle error to the range [-180, 180),
 * so it always takes the shortest rotational path to the setpoint.
 *
 * <p>Example:
 * <ul>
 *   <li>Setpoint = 10°</li>
 *   <li>Measurement = 350°</li>
 *   <li>Error = +20° (instead of -340°)</li>
 * </ul>
 */
public class AnglePIDController extends PIDController {

  /**
   * Creates an AnglePIDController with a default period of 0.02 seconds.
   *
   * @param kp proportional gain
   * @param ki integral gain
   * @param kd derivative gain
   */
  public AnglePIDController(double kp, double ki, double kd) {
    super(kp, ki, kd);

    // Automatically wrap angles from -180 to 180
    enableContinuousInput(-180.0, 180.0);
  }

  /**
   * Creates an AnglePIDController.
   *
   * @param kp proportional gain
   * @param ki integral gain
   * @param kd derivative gain
   * @param period loop period in seconds
   */
  public AnglePIDController(double kp, double ki, double kd, double period) {
    super(kp, ki, kd, period);

    // Automatically wrap angles from -180 to 180
    enableContinuousInput(-180.0, 180.0);
  }

  /**
   * Normalizes an angle to the range [-180, 180).
   *
   * @param angleDeg angle in degrees
   * @return normalized angle
   */
  public static double normalizeAngle(double angleDeg) {
    angleDeg %= 360.0;

    if (angleDeg >= 180.0) {
      angleDeg -= 360.0;
    }

    if (angleDeg < -180.0) {
      angleDeg += 360.0;
    }

    return angleDeg;
  }

  /**
   * Sets the setpoint angle in degrees.
   *
   * @param setpointDegrees desired angle
   */
  @Override
  public void setSetpoint(double setpointDegrees) {
    super.setSetpoint(normalizeAngle(setpointDegrees));
  }

  /**
   * Calculates the PID output using an angle measurement.
   *
   * @param measurementDegrees current angle
   * @return PID output
   */
  @Override
  public double calculate(double measurementDegrees) {
    return super.calculate(normalizeAngle(measurementDegrees));
  }

  /**
   * Calculates the PID output using an angle measurement and setpoint.
   *
   * @param measurementDegrees current angle
   * @param setpointDegrees desired angle
   * @return PID output
   */
  @Override
  public double calculate(double measurementDegrees, double setpointDegrees) {
    return super.calculate(
        normalizeAngle(measurementDegrees),
        normalizeAngle(setpointDegrees));
  }
}