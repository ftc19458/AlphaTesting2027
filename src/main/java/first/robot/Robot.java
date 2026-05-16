package first.robot;

import org.wpilib.drive.MecanumDrive;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.hardware.expansionhub.ExpansionHubMotor;
import org.wpilib.smartdashboard.SmartDashboard;

public class Robot extends OpModeRobot {
  private final ExpansionHubMotor frontLeft = new ExpansionHubMotor(0, 0);
  private final ExpansionHubMotor backLeft = new ExpansionHubMotor(0, 1);
  private final ExpansionHubMotor backRight = new ExpansionHubMotor(0, 2);
  private final ExpansionHubMotor frontRight = new ExpansionHubMotor(0, 3);


  public final MecanumDrive drive;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    if (isAutonomous()) {
      resetEncoders();
      drive = null;
      frontLeft.getPositionConstants().setPID(0.3, 0,0);
      backLeft.getPositionConstants().setPID(0.3, 0, 0);
      frontRight.getPositionConstants().setPID(0.3, 0, 0);
      backRight.getPositionConstants().setPID(0.3, 0, 0);
    } else {
      drive = new MecanumDrive(
        frontLeft::setThrottle,
        backLeft::setThrottle,
        frontRight::setThrottle,
        backRight::setThrottle
      );
    }

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

  //temporary using the drive encodrs until I setup odo
  public void setSetpoints(double x, double y){
    frontLeft.setPositionSetpoint(x + y);
    backLeft.setPositionSetpoint(x - y);
    frontRight.setPositionSetpoint(x - y );
    backRight.setPositionSetpoint(x + y);

  }

  public boolean isDoneMoving(int target, int tolerance) {


    return Math.abs(Math.abs(frontLeft.getEncoderPosition()) - target) <= tolerance &&
           Math.abs(Math.abs(backLeft.getEncoderPosition()) - target) <= tolerance &&
           Math.abs(Math.abs(frontRight.getEncoderPosition()) - target) <= tolerance &&
           Math.abs(Math.abs(backRight.getEncoderPosition()) - target) <= tolerance;
  }

  public void  updateDash(){
    double flPos = frontLeft.getEncoderPosition();
    double blPos = backLeft.getEncoderPosition();
    double frPos = frontRight.getEncoderPosition();
    double brPos = backRight.getEncoderPosition();


    SmartDashboard.putNumber("front left", flPos);
    SmartDashboard.putNumber("back left", blPos);
    SmartDashboard.putNumber("front right", frPos);
    SmartDashboard.putNumber("back right", brPos);
  }

  public void resetEncoders() {
    frontLeft.resetEncoder();
    backLeft.resetEncoder();
    backRight.resetEncoder();
    frontRight.resetEncoder();
  }
}