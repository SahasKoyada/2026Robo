package frc.robot.subsystems.Turret;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  private final SparkMax motor = new SparkMax(41, MotorType.kBrushless); // CAN ID 20
  private final RelativeEncoder encoder = motor.getEncoder();

  private static final double kMinRot = -1.0;  
  private static final double kMaxRot =  1.0;  

  public Turret() {
    encoder.setPosition(0.0); // zero at boot until I have an abs encoder
  }

  public double getMotorRotations() {
    return encoder.getPosition();
  }

  public void setDutyCycle(double output) {
    double pos = getMotorRotations();

    if ((pos <= kMinRot && output < 0) || (pos >= kMaxRot && output > 0)) {
      motor.set(0.0);
      return;
    }

    motor.set(MathUtil.clamp(output, -0.3, 0.3));
  }

  public void stop() {
    motor.set(0.0);
  }
}
