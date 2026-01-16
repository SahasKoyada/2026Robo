package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.util.LimelightHelpers;

public class HubLock extends Command {

  private static final String LL = "limelight"; 
  private static final int PIPELINE = 0;

  private static final Set<Integer> HUB_TAG_IDS =
      Set.of(2, 3, 4, 5, 8, 9, 10, 11);

  private final Turret turret;
  private PIDController rotController;

  private int lockedTagId = -1;

  public HubLock(Turret turret) {
    this.turret = turret;
    addRequirements(turret);

    rotController = new PIDController(0.02, 0.0, 0.001);
    rotController.setSetpoint(0.0);
    rotController.setTolerance(1.0);
  }

  @Override
  public void initialize() {
    System.out.println("HubLock INIT");

    LimelightHelpers.setPipelineIndex(LL, PIPELINE);
    rotController.reset();
    lockedTagId = -1;

    boolean tv = LimelightHelpers.getTV(LL);
    int id = (int) LimelightHelpers.getFiducialID(LL);

    System.out.println(
        "INIT | tv=" + tv +
        " id=" + id
    );

    if (tv && HUB_TAG_IDS.contains(id)) {
      lockedTagId = id;
      System.out.println("INIT | Locked onto tag " + lockedTagId);
    } else {
      System.out.println("INIT | No valid tag to lock");
    }
  }

  @Override
  public void execute() {
    boolean tv = LimelightHelpers.getTV(LL);
    int id = (int) LimelightHelpers.getFiducialID(LL);

    if (!tv) {
      System.out.println("EXEC | NO TARGET (tv=false)");
      rotController.reset();
      turret.stop();
      return;
    }

    if (!HUB_TAG_IDS.contains(id)) {
      System.out.println("EXEC | INVALID TAG ID: " + id);
      rotController.reset();
      turret.stop();
      return;
    }

    if (lockedTagId == -1) {
      lockedTagId = id;
      System.out.println("EXEC | Locking onto tag " + lockedTagId);
    }

    if (id != lockedTagId) {
      System.out.println(
          "EXEC | TAG CHANGED (locked=" + lockedTagId + " seen=" + id + ")"
      );
      rotController.reset();
      turret.stop();
      return;
    }

    double[] targetSpace = LimelightHelpers.getBotPose_TargetSpace(LL);

    double yawErrDeg = targetSpace[4];

    double output = -rotController.calculate(yawErrDeg);
    output = MathUtil.clamp(output, -0.25, 0.25);

    System.out.println(
        "EXEC | tag=" + id +
        " yawErrDeg=" + yawErrDeg +
        " pidOut=" + output
    );

    turret.setDutyCycle(output);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("HubLock END | interrupted=" + interrupted);
    rotController.reset();
    turret.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
