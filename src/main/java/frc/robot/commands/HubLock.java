package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.util.LimelightHelpers;

public class HubLock extends Command {
  private static final String LL = "limelight-tag";
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
    double yawErrorDeg = targetSpace[4]; // matches your AlignToReef pattern

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
