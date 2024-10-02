package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class passthrough extends SubsystemBase{
    private TalonFX pass_motor;

    public passthrough(TalonFX s){
        this.pass_motor = s;
    }
    public void setspeed(double speed){
        pass_motor.set(speed);
    }
}