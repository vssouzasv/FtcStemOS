package org.firstinspires.ftc.teamcode.auto2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

// Classe que usamos para inicializar o hardware do robô
public class HardwareInit {
    // Declaração dos objetos de motor
     DcMotor motorEsquerda   = null;
     DcMotor motorDireita  = null;
     DcMotor motorEsquerdaTras = null;
     DcMotor motorDireitaTras = null;
     // Array DcMotor para guardar todos motores em uma só variável
    DcMotor[] motores;
    // Objeto OpMode
    OpMode opMode;

    // Pegamos a referência do objeto OpMode de classes teleop e autônomo
    public HardwareInit(OpMode opMode) {
        this.opMode = opMode;
    }

    // Função que quando chamada inicia o hardware do robô
    public void iniciarHardware() {
        // Inicialização dos motores
        motorEsquerda  = opMode.hardwareMap.get(DcMotor.class, "motor_esquerda");
        motorDireita = opMode.hardwareMap.get(DcMotor.class, "motor_direita");
        motorEsquerdaTras  = opMode.hardwareMap.get(DcMotor.class, "motor_esquerdaTras");
        motorDireitaTras = opMode.hardwareMap.get(DcMotor.class, "motor_direitaTras");

        // Array de motores (opcinal)
        motores = new DcMotor[]{motorEsquerda, motorDireita, motorEsquerdaTras, motorDireitaTras};

        // Um dos lados do robô sempre tera motores invertidos, portanto revertar os motores aqui
        // vale dizer que uma transmissão por engranagens inverte a direção do movimento
        motorEsquerda.setDirection(DcMotor.Direction.REVERSE);
        motorEsquerdaTras.setDirection(DcMotor.Direction.REVERSE);
        motorDireita.setDirection(DcMotor.Direction.FORWARD);
        motorDireitaTras.setDirection(DcMotor.Direction.FORWARD);

    }

    // getter para objeto do opMode
    public OpMode getOpMode() {
        return opMode;
    }
}
