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
