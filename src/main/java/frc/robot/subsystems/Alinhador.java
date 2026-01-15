package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
  

public class Alinhador extends SubsystemBase{
    public SparkMax AlinhadorBoquinha;

    public static final double limiteSuperior = 90;
    public static final double limiteInferior = 5;
    public static final double Reducao = 12;
    private static final double VelocidadeMAX = 0.15;
    private static final double MargenErro = 10;
    public boolean usandoFF = false;

    private RelativeEncoder BoquinhaEncoder;
    private SparkClosedLoopController pid;
    private ArmFeedforward ff;

    public Alinhador(){
        AlinhadorBoquinha = new SparkMax(Constants.Alinhador.AlinhadorBoquinha, MotorType.kBrushless);
        BoquinhaEncoder = AlinhadorBoquinha.getEncoder();
        AlinhadorBoquinha.configure(
        new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

        SparkMaxConfig cfg = new SparkMaxConfig();

         cfg
         .idleMode(IdleMode.kBrake)
         .inverted(false)
         .smartCurrentLimit(60);

         cfg.closedLoop
         .p(0.3)
         .i(0.0)
         .d(0.0)
         .outputRange(-1.0, 1.0);

         AlinhadorBoquinha.configure(
         cfg, 
         ResetMode.kNoResetSafeParameters, 
         PersistMode.kPersistParameters);
    }
        public double getAngulo(){
        return BoquinhaEncoder.getPosition() * (360/ Reducao);
        }
        public double getAnguloRad(){
            return Math.toRadians(getAngulo());
        }
        public double grausParaRotacao(double graus){
            return (graus / 360) * Reducao;
        }
        public void moverParaAngulo(double alvoGraus){
            alvoGraus = MathUtil.clamp(alvoGraus, limiteInferior, limiteSuperior);
            usandoFF = true;

            double alvoCompensado = CompensarComFF(alvoGraus);
            pid.setReference(grausParaRotacao(alvoCompensado), ControlType.kPosition);
        }

        public boolean noLimiteSuperior(){
        return getAngulo() >= (limiteSuperior - MargenErro);
        }

        public Boolean noLimiteInferior(){
        return getAngulo() <= (limiteInferior - MargenErro);
        }

        public double getAlinhadorBoquinha(){
        return BoquinhaEncoder.getPosition();
        }

        public void setAlinhadorBoquinhaVelocidade(double velocidade){
            double LimiteVelocidade = MathUtil.clamp(velocidade, -VelocidadeMAX, +VelocidadeMAX);
            double Angulo = getAngulo();
           if (LimiteVelocidade > 0) {
             double distancia = limiteSuperior - Angulo;
              if(distancia <= 0) {
               AlinhadorBoquinha.set(0);
           return;
              }
              if (distancia < MargenErro) {
               LimiteVelocidade *= distancia/MargenErro;
              }
           }
           if (LimiteVelocidade < 0){
             double distancia = Angulo - limiteInferior;
             if(distancia <= 0){
               AlinhadorBoquinha.set(0);
           return;
             }
             if (distancia < MargenErro){
               LimiteVelocidade *= distancia / MargenErro;
             }
           }
           usandoFF = false;
           AlinhadorBoquinha.set(LimiteVelocidade);
        }
        private double CompensarComFF(double alvoGraus){
            double ffVolts = ff.calculate(Math.toRadians(alvoGraus), 0.0);
            double compensacao = MathUtil.clamp(ffVolts * 0.05, -5.0, 5.0);
            return alvoGraus + compensacao;
        }
        public void stopAlinhador(){
            AlinhadorBoquinha.set(0);
        }
        public void Parar(){
            AlinhadorBoquinha.set(0);
        }
        
  @Override
  public void periodic() {
  SmartDashboard.putNumber("IntaPke Ângulo (°)", getAngulo());
  SmartDashboard.putNumber("Boquinha Rotações",
  BoquinhaEncoder.getPosition());
  SmartDashboard.putBoolean("Limite Superior 205", noLimiteSuperior());
  SmartDashboard.putBoolean("Limite Inferior 90º", noLimiteInferior());
  
  SmartDashboard.putNumber("Lift Output", AlinhadorBoquinha.get());
  /*Manda as informação para visualização do driver com a função SmartDashBoard */
  }
}
