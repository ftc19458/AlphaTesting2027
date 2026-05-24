package first.robot;

import org.wpilib.drive.MecanumDrive;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.hardware.expansionhub.ExpansionHubMotor;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.smartdashboard.SmartDashboard;

public class Robot extends OpModeRobot {
  private final ExpansionHubMotor frontLeft = new ExpansionHubMotor(0, 0);
  private final ExpansionHubMotor backLeft = new ExpansionHubMotor(0, 1);
  private final ExpansionHubMotor backRight = new ExpansionHubMotor(0, 2);
  private final ExpansionHubMotor frontRight = new ExpansionHubMotor(0, 3);

  private int frontLeftSetpoint, backLeftSetpoint, frontRightSetpoint, backRightSetpoint;

  private static final OnboardIMU.MountOrientation orientation =
      OnboardIMU.MountOrientation.LANDSCAPE;

  private static OnboardIMU imu = new OnboardIMU(orientation);


  public MecanumDrive drive;

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

    imu.resetYaw();
  }

  public void setDrivePowers(double x, double y, double rotation) {
    drive.driveCartesian(x, y, rotation);
    
  }

  public void setDrivePowersFieldCentric(double x, double y, double rotation){
    drive.driveCartesian(x, y, rotation, imu.getRotation2d());
  }

  //temporary using the drive encodrs until I setup odo
  public void setSetpoints(int x, int y){
    frontLeftSetpoint = x + y;
    backLeftSetpoint = x - y;
    frontRightSetpoint = x - y;
    backRightSetpoint = x + y;


    frontLeft.setPositionSetpoint(frontLeftSetpoint);
    backLeft.setPositionSetpoint(backLeftSetpoint);
    frontRight.setPositionSetpoint(frontRightSetpoint);
    backRight.setPositionSetpoint(backRightSetpoint);
    

  }

  public void setPidConstants() {
       frontLeft.getPositionConstants().setPID(0.3, 0,0);
      backLeft.getPositionConstants().setPID(0.3, 0, 0);
      frontRight.getPositionConstants().setPID(0.3, 0, 0);
      backRight.getPositionConstants().setPID(0.3, 0, 0);
  }

  public boolean isDoneMoving(int tolerance) {

    return Math.abs(frontLeft.getEncoderPosition() - frontLeftSetpoint) <= tolerance &&
           Math.abs(backLeft.getEncoderPosition() - backLeftSetpoint) <= tolerance &&
           Math.abs(frontRight.getEncoderPosition() - frontRightSetpoint) <= tolerance &&
           Math.abs(backRight.getEncoderPosition()- backRightSetpoint) <= tolerance;
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

  public void initTelopDriving(){
         drive = new MecanumDrive(
        frontLeft::setThrottle,
        backLeft::setThrottle,
        frontRight::setThrottle,
        backRight::setThrottle
      );
  }

  public void resetEncoders() {
    frontLeft.resetEncoder();
    backLeft.resetEncoder();
    backRight.resetEncoder();
    frontRight.resetEncoder();
  }
}