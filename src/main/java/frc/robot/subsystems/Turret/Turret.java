package frc.robot.subsystems.Turret;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings({ "removal", "unused" })
public class Turret extends SubsystemBase {
  private static final int kMotorCanId = 41;

  private final SparkMax motor = new SparkMax(kMotorCanId, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();

  private static final double kMinRot = -1.0;
  private static final double kMaxRot =  1.0;
  private static final boolean kEnableSoftLimits = false;

  private static final double kDeadband = 0.02;
  private static final double kMaxDuty = 0.35;
  private static final double kMinDutyToMove = 0.08;

  private double lastPrintTimeSec = 0.0;
  private double lastCmdDuty = 0.0;

  public Turret() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    config.smartCurrentLimit(30);

    motor.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    encoder.setPosition(0.0);
    System.out.println("[Turret] init: CAN=" + kMotorCanId + " inverted=false brake=true");
  }

  public double getMotorRotations() {
    return encoder.getPosition();
  }

  public double getMotorRpm() {
    return encoder.getVelocity();
  }


  public void setDutyCycle(double duty) {
    final double posRot = getMotorRotations();

    double cmd = MathUtil.applyDeadband(duty, kDeadband);
    cmd = MathUtil.clamp(cmd, -kMaxDuty, kMaxDuty);

    if (Math.abs(cmd) > 1e-6) {
      cmd = Math.copySign(Math.max(Math.abs(cmd), kMinDutyToMove), cmd);
    }

    if (kEnableSoftLimits) {
      boolean hitMin = posRot <= kMinRot && cmd < 0;
      boolean hitMax = posRot >= kMaxRot && cmd > 0;
      if (hitMin || hitMax) {
        motor.set(0.0);
        rateLimitedPrint(
            "[Turret] SOFT LIMIT hit (posRot=" + fmt(posRot) + ") cmd=" + fmt(cmd) +
                " min=" + fmt(kMinRot) + " max=" + fmt(kMaxRot));
        lastCmdDuty = 0.0;
        return;
      }
    }

    motor.set(cmd);
    lastCmdDuty = cmd;

    rateLimitedPrint(
        "[Turret] cmd=" + fmt(cmd) +
            " in=" + fmt(duty) +
            " posRot=" + fmt(posRot) +
            " rpm=" + fmt(getMotorRpm()));
  }

  public void stop() {
    motor.set(0.0);
    lastCmdDuty = 0.0;
    rateLimitedPrint("[Turret] stop");
  }

  private void rateLimitedPrint(String msg) {
    double now = Timer.getFPGATimestamp();
    if (now - lastPrintTimeSec > 0.25) {
      System.out.println(msg);
      lastPrintTimeSec = now;
    }
  }

  private static String fmt(double v) {
    return String.format("%.3f", v);
  }
}



/* 
package frc.robot.subsystems.Turret;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase;

@SuppressWarnings("removal")



public class Turret extends SubsystemBase {
  private final SparkMax motor = new SparkMax(41, MotorType.kBrushless); // CAN ID 20
  private final RelativeEncoder encoder = motor.getEncoder();

  private static final double kMinRot = -1.0;  
  private static final double kMaxRot =  1.0;  
  private static final boolean kEnableSoftLimits = false; // TEMP: disable until abs encoder


public Turret() {
  SparkMaxConfig config = new SparkMaxConfig();
  config.idleMode(IdleMode.kBrake);
  config.inverted(false);

  motor.configure(
      config,
      SparkBase.ResetMode.kResetSafeParameters,
      SparkBase.PersistMode.kPersistParameters
  );

  encoder.setPosition(0.0);
}


  public double getMotorRotations() {
    return encoder.getPosition();
  }

public void setDutyCycle(double output) {
  double pos = getMotorRotations();

  if (kEnableSoftLimits) {
    if ((pos <= kMinRot && output < 0) || (pos >= kMaxRot && output > 0)) {
      motor.set(0.0);
      return;
    }
  }

  motor.set(MathUtil.clamp(output, -0.05, 0.05));
}


  public void stop() {
    motor.set(0.0);
  }
}
*/