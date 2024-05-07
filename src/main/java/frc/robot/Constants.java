package frc.robot;

public class Constants {

    public static final class OI_CONSTANTS {

        public static final int DRIVER_ID = 0;
        public static final int OPERADOR_ID = 1;

    }

    public static final class WRIST_CONSTANTS {

        public static final int id = 33;

        public static final double kP = 0.003; //0.003
        public static final double kI = 0.001; //0.002
        public static final double kD = 0; 

        public static final double kP_S = 0.01; //0.003
        public static final double kI_S = 0.006; //0.002
        public static final double kD_S = 0; 


        public static final double INTAKING_POSE = 154; // 154
        public static final double REPOSO_POSE = 0;
        public static final double SHOOTER_POSE = 136;

    }

    public static final class HOMBRO_CONTSTANTS {

        public static final int HOMBRO_IZQ_ID = 20;
        public static final int HOMBRO_DER_ID= 21;

        public static final double kP = 0.005;
        public static final double kI = 0.0003;
        public static final double kD = 0;

        public static final double kP_C = 0.005;

        public static final double kBrazoMaxVelocity = 0.5;
        public static final double kBrazoMaxAcceleration = 0.5;


        public static final double kS = 0.25;
        public static final double kG = 0.64;
        public static final double kV = 1.13;
        public static final double kA = 0.02;


        public static final double AMP_POSE = 90;

    }

    public static final class INTAKE_CONSTANTS {

        public static final int MOTOR_UP_INTAKE_ID = 13;
        public static final int MOTOR_DOWN_INTAKE_ID = 11;

        public static final double INTAKING_VELOCITY = 0.3;

    }

    public static final class ELEVADOR_CONSTANTS{

        public static final int ELEVADOR_LEFT_ID = 22;
        public static final int ELEVADOR_RIGHT_ID = 23;

        public static final double kP = 0.04; //0.35
        public static final double kI = 0.002; //0.001
        public static final double kD = 0;

    }

    public static final class SHOOTER_CONSTANTS{

        public static final int SHOOTER_DER_ID = 14;
        public static final int SHOOTER_IZQ_ID =  12;

        public static final double kP = 0;
        public static final double kI = 0.05;
        public static final double kD = 0;

        public static final double VELOCITYFORAMP = 2;
        public static final double VELOCITYFORSPEAKER_R = 15; //15
        public static final double VELOCITYFORSPEAKER_L = 20; //20

        //Valores Bonitos Corta
        //15 
        //20

        //Valores Bonitos Larga
        //20
        //25

    }

    public static final class TARGETS_MEASURES {

        public static final double TARGET_SPEAKER = 2.04;  
    
    }
    
}
