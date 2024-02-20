package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Controller;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class TeleopDriver extends Command {

  private final Vision m_vision;
  private final Swerve s_Swerve;
  private final Controller m_controller;
  
  private XboxController driver;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private boolean onePress1 = false;
  private boolean onePress2 = false;
  private boolean onepressAuto = false;
  private boolean onepressLED = false;

  private double translationVal;
  private double strafeVal;
  private double rotationVal;

  private int counter = 0;

  public TeleopDriver(Swerve s_Swerve, Vision s_Vision, Controller s_Controller) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    m_vision = s_Vision;
    addRequirements(m_vision);
    m_controller = s_Controller;
    addRequirements(m_controller);
    driver = m_controller.getDriverController();
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    if (Constants.Swerve.slow) {
      translationVal =
          translationLimiter.calculate(
              MathUtil.applyDeadband(driver.getLeftY() * Constants.Swerve.slowRegulator, Constants.Swerve.axisDeadBand));
      strafeVal =
          strafeLimiter.calculate(
              MathUtil.applyDeadband(driver.getLeftX() * Constants.Swerve.slowRegulator, Constants.Swerve.axisDeadBand));
      if(Constants.SuperStructure.isAuto && (Constants.Vision.id == 4 || Constants.Vision.id == 7)) {
          rotationVal = s_Swerve.calculateAutoFacing();
      } else {
          rotationVal =
              rotationLimiter.calculate(
                  MathUtil.applyDeadband(driver.getRightX() * Math.pow(Constants.Swerve.slowRegulator, 2), Constants.Swerve.axisDeadBand));
      }
    } else {
      translationVal =
          translationLimiter.calculate(
              MathUtil.applyDeadband(driver.getLeftY(), Constants.Swerve.axisDeadBand));
      strafeVal =
          strafeLimiter.calculate(
              MathUtil.applyDeadband(driver.getLeftX(), Constants.Swerve.axisDeadBand));
      if(Constants.SuperStructure.isAuto && (Constants.Vision.id == 4 || Constants.Vision.id == 7)){
          rotationVal = s_Swerve.calculateAutoFacing();
      } else {
          rotationVal =
              rotationLimiter.calculate(
                  MathUtil.applyDeadband(driver.getRightX() * Constants.Swerve.slowRegulator, Constants.Swerve.axisDeadBand));
      }
    }
    
    if (driver.getLeftBumperPressed() && onePress1 == false && Constants.SuperStructure.isAuto == false) {
      Constants.Swerve.fieldOriented = !Constants.Swerve.fieldOriented;
      onePress1 = true;
    }else if (driver.getLeftBumperReleased()) {
      onePress1 = false;
    }

    if (driver.getRightBumperPressed() && onePress2 == false) {
      Constants.Swerve.slow = !Constants.Swerve.slow;
      onePress2 = true;
    }else if (driver.getRightBumperReleased()) {
      onePress2 = false;
    }

    if (driver.getBackButton()) s_Swerve.zeroGyro();
    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity, Constants.Swerve.fieldOriented,
        true);

    if(driver.getAButtonPressed() && onepressAuto == false) {
      Constants.SuperStructure.isAuto = !Constants.SuperStructure.isAuto;
      onepressAuto = true;
    }
    if(driver.getAButtonReleased()) onepressAuto = false;
    if(driver.getXButtonPressed() && onepressLED == false) {
      switch(counter) {
        case 0:
          m_vision.setLEDMode(counter);
          counter++;
          break;
        case 1:
          m_vision.setLEDMode(counter);
          counter++;
          break;
        case 2:
          m_vision.setLEDMode(counter);
          counter++;
          break;
        case 3:
          m_vision.setLEDMode(counter);
          counter = 0;
          break;
      }
      onepressLED = true;
    }
    if(driver.getXButtonReleased()) onepressLED = false;

    //testing motors
    // if(driver.getPOV() == 0) s_Swerve.mSwerveMods[0].setDriveMotor(1);
    // if(driver.getPOV() == 45) s_Swerve.mSwerveMods[0].setAngleMotor(0.3);
    // if(driver.getPOV() == 90) s_Swerve.mSwerveMods[1].setDriveMotor(1);
    // if(driver.getPOV() == 135) s_Swerve.mSwerveMods[1].setAngleMotor(0.3);
    // if(driver.getPOV() == 180) s_Swerve.mSwerveMods[2].setDriveMotor(1);
    // if(driver.getPOV() == 225) s_Swerve.mSwerveMods[2].setAngleMotor(0.3);
    // if(driver.getPOV() == 270) s_Swerve.mSwerveMods[3].setDriveMotor(1);
    // if(driver.getPOV() == 315) s_Swerve.mSwerveMods[3].setAngleMotor(0.3);

  }
}