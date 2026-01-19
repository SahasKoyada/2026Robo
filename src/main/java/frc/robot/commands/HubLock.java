package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.LimelightHelpers;



@SuppressWarnings("unused")
public class HubLock extends Command {
  private final Turret turret;
  private final Vision vision;
  private final int cameraIndex;

  private static final double kP = 0.02;        
  private static final double kMaxDuty = 0.30;
  private static final double kMinDuty = 0.09;  
  private static final double kDeadbandDeg = 0.7;

  private double lastPrint = 0.0;

  public HubLock(Turret turret, Vision vision, int cameraIndex) {
    this.turret = turret;
    this.vision = vision;
    this.cameraIndex = cameraIndex;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    boolean hasTarget = vision.hasTarget(cameraIndex);
    double txDeg = vision.getTxDeg(cameraIndex);

    double duty = 0.0;
    try {
    if (hasTarget) {
      double err = MathUtil.applyDeadband(txDeg, kDeadbandDeg);

      //inverter
      duty = kP * err;

      duty = MathUtil.clamp(duty, -kMaxDuty, kMaxDuty);
      if (Math.abs(duty) > 1e-6) {
        duty = Math.copySign(Math.max(Math.abs(duty), kMinDuty), duty);
      }
    
    double distMeters = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-tag")
      .getTranslation()
      .getNorm();
    turret.enableAngler(true);
    turret.setAnglerDistanceMeters(distMeters);
    } else {

    turret.enableAngler(false);
  }
  
   

    turret.setDutyCycle(duty);

    double now = Timer.getFPGATimestamp();
    if (now - lastPrint > 0.25) {
      System.out.println("[HubLock] hasTarget=" + hasTarget
          + " txDeg=" + String.format("%.2f", txDeg)
          + " duty=" + String.format("%.3f", duty));
      lastPrint = now;
    }
  } catch (Exception e) {
      System.out.println("hi");}
      
    }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
    turret.enableAngler(false);
    System.out.println("[HubLock] end interrupted=" + interrupted);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}





/*package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.util.LimelightHelpers;

public class HubLock extends Command {
  private static final String LL = "limelight";
  private static final int PIPELINE = 0;

  private static final Set<Integer> HUB_TAG_IDS = Set.of(2, 3, 4, 5, 8, 9, 10, 11);

  private final Turret turret;
  private final PIDController pid = new PIDController(0.02, 0.0, 0.001);

  private int lockedId = -1;

  public HubLock(Turret turret) {
    this.turret = turret;
    addRequirements(turret);

    pid.setSetpoint(0.0);
    pid.setTolerance(1.0);
  }

  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex(LL, PIPELINE);
    pid.reset();
    lockedId = -1;
  }

  @Override
  public void execute() {
    boolean tv = LimelightHelpers.getTV(LL);
    int id = (int) LimelightHelpers.getFiducialID(LL);

    if (!tv) {
      pid.reset();
      turret.stop();
      return;
    }

    if (!HUB_TAG_IDS.contains(id)) {
      pid.reset();
      turret.stop();
      return;
    }

    if (lockedId == -1) lockedId = id;
    if (id != lockedId) {
      pid.reset();
      turret.stop();
      
      return;
    }

    double[] targetSpace = LimelightHelpers.getBotPose_TargetSpace(LL);
    double yawErrorDeg = targetSpace[4]; 
    System.out.println("HubLock | id=" + id + " yawErr=" + yawErrorDeg);

    double out = pid.calculate(yawErrorDeg);

    out = MathUtil.clamp(out, -0.25, 0.25);
    turret.setDutyCycle(out);
  }

  @Override
  public void end(boolean interrupted) {
    pid.reset();
    turret.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
*/