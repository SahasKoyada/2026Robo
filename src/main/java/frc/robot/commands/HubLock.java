package frc.robot.commands;
import java.util.Set;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.util.LimelightHelpers;

public class HubLock extends Command {
  private static final Set<Integer> HUB_TAG_IDS = Set.of(2, 3, 4, 5, 8, 9, 10, 11);

  private final Turret turret;
  private final PIDController pid = new PIDController(0.02, 0.0, 0.001);

  public HubLock(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
    pid.setSetpoint(0.0);
    pid.setTolerance(1.0);
  }

  @Override
  public void execute() {
    System.out.println(
  "HubLocking | tv=" + LimelightHelpers.getTV("limelight-tag") +
  " id=" + (int) LimelightHelpers.getFiducialID("limelight-tag") +
  " tx=" + LimelightHelpers.getTX("limelight-tag")
);

    if (!LimelightHelpers.getTV("limelight-tag")) {
      turret.stop();
      return;
    }

    int id = (int) LimelightHelpers.getFiducialID("limelight-tag");
    if (!HUB_TAG_IDS.contains(id)) {
      turret.stop();
      return;
    }

    double tx = LimelightHelpers.getTX("limelight-tag"); // degrees
    double out = pid.calculate(tx);

    if (Math.abs(tx) < 1.0) out = 0.0;
    out = MathUtil.clamp(out, -0.25, 0.25);

    turret.setDutyCycle(out);
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
