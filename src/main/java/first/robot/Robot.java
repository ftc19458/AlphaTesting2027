package first.robot;

import org.wpilib.drive.MecanumDrive;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.hardware.expansionhub.ExpansionHubMotor;

public class Robot extends OpModeRobot {
  private final ExpansionHubMotor frontLeft = new ExpansionHubMotor(0, 0);
  private final ExpansionHubMotor backLeft = new ExpansionHubMotor(0, 1);
  private final ExpansionHubMotor backRight = new ExpansionHubMotor(0, 2);
  private final ExpansionHubMotor frontRight = new ExpansionHubMotor(0, 3);

  public final MecanumDrive drive = new MecanumDrive(
    frontLeft::setThrottle, backLeft::setThrottle, frontRight::setThrottle, backRight::setThrottle
  );

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    frontLeft.setReversed(true);
    backLeft.setReversed(true);

    // tried setting these to both true and false, the drivetrain drifted either way
    frontLeft.setFloatOn0(false);
    backLeft.setFloatOn0(false);
    backRight.setFloatOn0(false);
    frontRight.setFloatOn0(false);
  }

  public void setDrivePowers(double x, double y, double rotation) {
    drive.driveCartesian(x, y, rotation);
  }
}