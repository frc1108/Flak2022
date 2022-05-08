// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.util.InterpolatingTreeMap;

public class ShootWhileMove extends CommandBase {
  private final Shooter m_shooter;
  //private final Turret m_turret;
  private final Drivetrain m_drive;
  //private final ShooterHood m_hood;
  private final boolean m_updatePose;
  private final ColorSensor m_color;
  private final XboxController m_driver;
  private double m_wrongBallTime;
  private final Timer m_timer = new Timer();
  private final InterpolatingTreeMap m_timeTable = new InterpolatingTreeMap<>();

  private static Point2D[] m_shotTimes = 
      new Point2D.Double[]{
          //(dist,time)
          new Point2D.Double(105,0.82), 
          new Point2D.Double(135,0.82), 
          new Point2D.Double(165,0.85),//
          new Point2D.Double(195,0.85),
          new Point2D.Double(250,1.05),
          //
      };
  private static LinearInterpolationTable m_timeTable = new LinearInterpolationTable(m_shotTimes);

  private static LinearInterpolationTable m_hoodTable = ShooterConstants.khoodTable;
  private static LinearInterpolationTable m_rpmTable = ShooterConstants.krpmTable;
  

  public ShootWhileMove(Shooter shooter, Turret turret, Drivetrain drive,ShooterHood hood, boolean updatePose, ColorSensor color, XboxController driver){
      m_shooter = shooter;
      m_turret = turret;
      m_drive = drive;
      m_hood = hood;
      m_updatePose = updatePose;
      m_color = color;
      m_driver = driver;
      addRequirements(shooter, turret, hood);

      m_timeTable.put(105,0.82);
      
  }
  public ShootWhileMove(Shooter shooter, Turret turret, Drivetrain drive,ShooterHood hood, boolean updatePose, ColorSensor color){
      m_shooter = shooter;
      m_turret = turret;
      m_drive = drive;
      m_hood = hood;
      m_updatePose = updatePose;
      m_color = color;
      m_driver = new XboxController(4);
      addRequirements(shooter, turret, hood);
  }
  @Override
  public void initialize(){
      m_turret.enable();
      m_turret.trackTarget(true);
      m_timer.reset();
      m_timer.start();
      SmartDashboard.putNumber("SetHoodAdjust", 0.0);
      SmartDashboard.putNumber("SetShotAdjust", 0);
      SmartDashboard.putBoolean("Adjust Shot?", false);
      m_wrongBallTime = Double.NEGATIVE_INFINITY;
  }

  @Override
  public void execute(){

      double currentTime = m_timer.get();        
      boolean wrongBall = m_color.isWrongBall();
      if(wrongBall){
          m_wrongBallTime = currentTime;
      }

      SmartDashboard.putNumber("Current Time", currentTime);

      SmartDashboard.putBoolean("Wrong Ball", wrongBall);

      SmartDashboard.putBoolean("Shooter Running", true);

      FieldRelativeSpeed robotVel = m_drive.getFieldRelativeSpeed();
      FieldRelativeAccel robotAccel = m_drive.getFieldRelativeAccel();

      Translation2d target = GoalConstants.kGoalLocation;

      if(currentTime <= m_wrongBallTime+0.100){
          target = GoalConstants.kWrongBallGoal;
      }

      Translation2d robotToGoal = target.minus(m_drive.getPose().getTranslation());
      double dist = robotToGoal.getDistance(new Translation2d())*39.37;

      SmartDashboard.putNumber("Calculated (in)", dist);

      double fixedShotTime = m_timeTable.getOutput(dist);

      double virtualGoalX = target.getX()-fixedShotTime*(robotVel.vx+robotAccel.ax*ShooterConstants.kAccelCompFactor);
      double virtualGoalY = target.getY()-fixedShotTime*(robotVel.vy+robotAccel.ay*ShooterConstants.kAccelCompFactor);

      SmartDashboard.putNumber("Goal X", virtualGoalX);
      SmartDashboard.putNumber("Goal Y", virtualGoalY);

      Translation2d movingGoalLocation = new Translation2d(virtualGoalX,virtualGoalY);

      Translation2d toMovingGoal = movingGoalLocation.minus(m_drive.getPose().getTranslation());

      double newDist = toMovingGoal.getDistance(new Translation2d())*39.37;

      m_turret.aimAtGoal(m_drive.getPose(), movingGoalLocation, false);
      
          if(SmartDashboard.getBoolean("Adjust Shot?", false)){
              m_shooter.run(ShooterConstants.krpmTable.getOutput(newDist)+SmartDashboard.getNumber("SetShotAdjust", 0));
              m_hood.run(m_hoodTable.getOutput(newDist)+SmartDashboard.getNumber("SetHoodAdjust", 0));
          }
          else{
              m_shooter.run(m_rpmTable.getOutput(newDist));
              m_hood.run(m_hoodTable.getOutput(newDist));
              
          }


      if(currentTime > 0.250 && Limelight.valid()){
          double dL = Limelight.getDistance()*0.0254;
          double tR = m_drive.getGyro().getRadians();
          double tT = m_turret.getMeasurement()-Math.PI;
          double tL = -1.0*Limelight.tx();
  
          Pose2d pose = calcPoseFromVision(dL, tR, tT, tL, GoalConstants.kGoalLocation);
  
          if(m_updatePose){
              m_drive.setPose(pose);
          }
  
      }

      if(m_turret.desiredInDeadzone()){
          m_driver.setRumble(RumbleType.kLeftRumble, 1.0);
          m_driver.setRumble(RumbleType.kRightRumble, 1.0);
      }
      else{
          m_driver.setRumble(RumbleType.kLeftRumble, 0.0);
          m_driver.setRumble(RumbleType.kRightRumble, 0.0);
      }

  }

  @Override
  public void end(boolean interrupted) {
      SmartDashboard.putBoolean("Shooter Running", false);
    m_turret.trackTarget(false);
    m_turret.disable();
    m_turret.stop();
    m_shooter.stop();
    m_hood.stop();
    m_timer.stop();
    m_driver.setRumble(RumbleType.kLeftRumble, 0.0);
    m_driver.setRumble(RumbleType.kRightRumble, 0.0);
  }
  
  private Pose2d calcPoseFromVision(double dL, double tR, double tT, double tL, Translation2d goal){
      double tG = tR+tT+tL;
      double rX = goal.getX()-dL*Math.cos(tG);
      double rY = goal.getY()-dL*Math.sin(tG);
  
      return new Pose2d(rX,rY, new Rotation2d(-tR));
  }

}