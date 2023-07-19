/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Autonomo 4 motores", group="Robot")
public class AutonomoEncoder extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         motorEsquerda   = null;
    private DcMotor         motorDireita  = null;
    private DcMotor motorEsquerdaTras = null;
    private DcMotor motorDireitaTras = null;
    DcMotor[] motores;

    private final ElapsedTime     runtime = new ElapsedTime();

    /*
     * Aqui setamos as variáveis para configurar nossa tração no período autônomo
     * precisamos desses valores para calcular a contagem necessária dos encoders para
     * a distância dada por nós, por exemplo, eu quero que o robô se mova 5 metros, entretanto
     * meu encoder somente faz a leitura por ticks, então, quanto seria 5 metros em ticks
     * Para isso que temos esses valores.
     * VALORES DE COMPRIMENTO EM POLEGADAS
     */
    static final double     COUNTS_PER_MOTOR_REV    = 1440; // CPR do motor, entre no site da fabricante
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;  // Redução entre motor e roda
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;  // Diâmetro da roda
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415); // Fator de conversão
    static final double     DRIVE_SPEED             = 0.6; // Com que tensão os motores se moverão para frente
    static final double     TURN_SPEED              = 0.5; // Com que tensão os motores farão o robô girar

    @Override
    public void runOpMode() {

        // Inicializando os 4 motores da tração
        motorEsquerda  = hardwareMap.get(DcMotor.class, "motor_esquerda");
        motorDireita = hardwareMap.get(DcMotor.class, "motor_direita");
        motorEsquerdaTras  = hardwareMap.get(DcMotor.class, "motor_esquerdaTras");
        motorDireitaTras = hardwareMap.get(DcMotor.class, "motor_direitaTras");

        // Aqui é algo não opcinal, colocar os 4 motores em um array DcMotor faz com que
        // comandos iguais para os 4 motores possa ser simplificado usando funções com loops for
        motores = new DcMotor[]{motorEsquerda, motorDireita, motorEsquerdaTras, motorDireitaTras};

        // Um dos lados do robô sempre tera motores invertidos, portanto revertar os motores aqui
        // vale dizer que uma transmissão por engranagens inverte a direção do movimento
        motorEsquerda.setDirection(DcMotor.Direction.REVERSE);
        motorEsquerdaTras.setDirection(DcMotor.Direction.REVERSE);
        motorDireita.setDirection(DcMotor.Direction.FORWARD);
        motorDireitaTras.setDirection(DcMotor.Direction.FORWARD);

        // Reseta os encoder antes de começar a movimentação
        setarModos(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setarModos(DcMotor.RunMode.RUN_USING_ENCODER);

        // Coloca a posição dos motores após resetados na telemetria da Drive Station
        printPosicaoMotores();
        telemetry.update();

        // Espera o botão de Start ser apertado na Drive Station
        waitForStart();

        // Aqui colocamos os movimentos que o robô fara
        encoderDrive(DRIVE_SPEED,  4,  4, 5.0);
        encoderDrive(TURN_SPEED,  -4,  4, 5.0);

        // Adiciona uma mensagem para quando o caminho terminar
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     * Aqui configuramos a movimentação do robô
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        // Variáveis para o setPoint dos motores
        int newLeftTarget;
        int newRightTarget;

        // Caso ainda tenhamos a DS rodando o código
        if (opModeIsActive()) {

            // Determina para que posição (em ticks) o robô deve se mover
            newLeftTarget = motorEsquerda.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = motorDireita.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            // Seta o setPoint para nossos motores
            motorEsquerda.setTargetPosition(newLeftTarget);
            motorEsquerdaTras.setTargetPosition(newLeftTarget);
            motorDireita.setTargetPosition(newRightTarget);
            motorDireitaTras.setTargetPosition(newRightTarget);

            // Define os motores para ir até a posição
            setarModos(DcMotor.RunMode.RUN_TO_POSITION);

            // Reseta o tempo
            runtime.reset();

            // Define a potência dos motores
            setPowers(Math.abs(speed));

            // Loop que verificara se o tempo de segurança se esgotou e se os motores
            // ainda estão inda para a posição designada
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorEsquerda.isBusy() && motorDireita.isBusy()) &&
                    (motorEsquerdaTras.isBusy() && motorDireitaTras.isBusy())) {

                // Telemetria da posição dos motores
                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                printPosicaoMotores();
                telemetry.update();
            }
            // Coloca os motores para parar e coloca para correr com o encoder novamente
            setPowers(0);
            setarModos(DcMotor.RunMode.RUN_USING_ENCODER);

            // Pausa opcional após terminar o movimento
            sleep(250);
        }
    }
    public void printPosicaoMotores() {
        telemetry.addData("Motores frontais",  " em %7d :%7d",
                motorEsquerda.getCurrentPosition(), motorDireita.getCurrentPosition());
        telemetry.addData("Motores de trás",  " em %7d :%7d",
                motorEsquerdaTras.getCurrentPosition(), motorDireitaTras.getCurrentPosition());
    }
    public void setarModos(DcMotor.RunMode mode) {
        for(DcMotor motors : motores) {
            motors.setMode(mode);
        }
    }
    public void setPowers(double speed) {
        for(DcMotor motors : motores) {
            motors.setPower(speed);
        }
    }
}