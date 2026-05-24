package first.robot;

import org.wpilib.driverstation.Gamepad;
import org.wpilib.math.util.MathUtil;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

@Teleop
public class MecanumTeleop extends PeriodicOpMode {
  private final Robot robot;
  private final Gamepad gamepad = new Gamepad(0);
  private final float MAX_SPEED = 0.75f;
  private final float DEADBAND = 0.1f;

  /** The Robot instance is passed into the opmode via the constructor. */
  public MecanumTeleop(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void periodic() {

    robot.setDrivePowers(
      -MathUtil.applyDeadband(gamepad.getLeftY(), DEADBAND) * MAX_SPEED,
      MathUtil.applyDeadband(gamepad.getLeftX(), DEADBAND) * MAX_SPEED,
      MathUtil.applyDeadband(gamepad.getRightX(), DEADBAND) * MAX_SPEED
    );
  }
  @Override
  public void start() {
  }
}