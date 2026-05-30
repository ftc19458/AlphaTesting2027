package first.robot;

import static org.wpilib.units.Units.Inches;

import java.util.function.DoubleConsumer;

import org.wpilib.drive.MecanumDrive;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.hardware.bus.I2C.Port;
import org.wpilib.hardware.hal.CANBusMap;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.units.measure.Distance;

import com.revrobotics.spark.A301;

import first.robot.controllers.AnglePIDController;
import first.robot.sensors.GoBildaPinpoint;
import first.robot.sensors.GoBildaPinpoint.EncoderDirection;

public class Robot extends OpModeRobot {
  private final A301 frontLeft = new A301(CANBusMap.CAN_D7);
  private final A301 backLeft = new A301(CANBusMap.CAN_D3);
  private final A301 backRight = new A301(CANBusMap.CAN_D0);
  private final A301 frontRight = new A301(CANBusMap.CAN_D1);


  private static final OnboardIMU.MountOrientation orientation =
      OnboardIMU.MountOrientation.LANDSCAPE;  

  private static OnboardIMU imu = new OnboardIMU(orientation);

  private final GoBildaPinpoint pinpoint = new GoBildaPinpoint(Port.PORT_1);


  private static double Tp = 0;//0.2;
  private static double Ti = 0;
  private static double Td = 0.0;//0.005;

  private static double Hp = 0.05;
  private static double Hi = 0;
  private static double Hd = 0.003;

  private PIDController xPid = new PIDController(Tp, Ti, Td);
  private PIDController yPid = new PIDController(Tp, Ti, Td);
  private PIDController hPid = new AnglePIDController(Hp, Hi, Hd);
  

  private Pose2d setpoint = Pose2d.kZero;



  public MecanumDrive drive;

  public DoubleConsumer velSetter(A301 motor, double maxVel, boolean inverted){
    return (vel) -> {
      double setVel = vel * maxVel;
      if(inverted){
        setVel = -setVel;
      }
      motor.setVoltage(maxVel);
    };
  }



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {


    imu.resetYaw();


             drive = new MecanumDrive(
      velSetter(frontLeft, 12, false),
      velSetter(backLeft, 12, false),
      velSetter(frontRight, 12, false),
      velSetter(backRight, 12, false)
      );

        pinpoint.setEncoderDirections(EncoderDirection.FORWARD, EncoderDirection.FORWARD);
        pinpoint.setEncoderResolution(19.8943678865);
        pinpoint.setOffsetsMM(-139.7,-25.4);
        pinpoint.resetPosAndIMU();
  }

  public void setDrivePowers(double x, double y, double rotation) {
    SmartDashboard.putNumber("left front current ", frontLeft.getOutputCurrent().get());
    SmartDashboard.putNumber("left back current ", backLeft.getOutputCurrent().get());
    SmartDashboard.putNumber("right front current ", frontRight.getOutputCurrent().get());
    SmartDashboard.putNumber("right back current ", backRight.getOutputCurrent().get());
    SmartDashboard.putNumber("Total current ", frontLeft.getOutputCurrent().get() + backLeft.getOutputCurrent().get() + frontRight.getOutputCurrent().get() + backRight.getOutputCurrent().get());

    SmartDashboard.putNumber("left front vel", frontLeft.getEncoderVelocity().get());
    SmartDashboard.putNumber("left back vel", backLeft.getEncoderVelocity().get());
    SmartDashboard.putNumber("right front vel", frontRight.getEncoderVelocity().get());
    SmartDashboard.putNumber("right back vel", backRight.getEncoderVelocity().get());



    drive.driveCartesian(x, y, rotation);
    
  }

  public void setDrivePowersFieldCentric(double x, double y, double rotation){
    drive.driveCartesian(x, y, rotation, imu.getRotation2d());
  }

  
  public void setSetpoint(double x, double y, double h){

    Distance xDist =  Distance.ofRelativeUnits(x, Inches);
    Distance yDist =  Distance.ofRelativeUnits(y, Inches);
    Rotation2d rot = Rotation2d.fromDegrees(h);

    setSetpoint(new Pose2d(xDist,yDist,rot)); 
  }

  public void setSetpoint(Pose2d pose){
    setpoint = pose;
    xPid.setSetpoint(pose.getMeasureX().in(Inches));
    yPid.setSetpoint(pose.getMeasureY().in(Inches));
    hPid.setSetpoint(pose.getRotation().getDegrees());
  }

  public Pose2d getSetpoint(){
    return setpoint;
  }

  public void updatePowers(){
    Pose2d pos = pinpoint.getPose();
    double xPower = -xPid.calculate(pos.getMeasureX().in(Inches));
    double yPower = yPid.calculate(pos.getMeasureY().in(Inches));
    double hPower = -hPid.calculate(pos.getRotation().getDegrees());

    setDrivePowersFieldCentric(xPower,yPower, hPower);
  }
  
  public Pose2d getCurPose2d(){
    return pinpoint.getPose();
  }

  public void updatePinpont(){
    pinpoint.update();
  }

  public void resetPinpoint()
  {
    pinpoint.resetPosAndIMU();
  }
}