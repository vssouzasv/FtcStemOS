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

package org.firstinspires.ftc.teamcode.auto2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// Classe que usamos para movimentar o robô no período autônomo
@Autonomous(name = "Autonomo Robot", group = "Robot")
public class AutonomoEncoderRobot extends LinearOpMode {
    // Objeto que usamos para acessar os motores
    HardwareInit hardware = new HardwareInit(this);
    // Utilizamos o objeto abaixo para contar o tempo decorrido desde o inicio da partida
    private final ElapsedTime runtime = new ElapsedTime();

    // Função OpMode que roda quando iniciamos o código na Driver Station
    @Override
    public void runOpMode() {
        // chamamos a função que inicia o hardware do robô
        hardware.iniciarHardware();
        // Reseta os encoder antes de começar a movimentação
        setarModos(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setarModos(DcMotor.RunMode.RUN_USING_ENCODER);

        // Coloca a posição dos motores após resetados na telemetria da Drive Station
        printPosicaoMotores();
        telemetry.update();

        // Espera o botão de Start ser apertado na Drive Station
        waitForStart();

        // Aqui colocamos os movimentos que o robô fara
        encoderDrive(Constants.DRIVE_SPEED, 4, 4, 5.0);
        encoderDrive(Constants.TURN_SPEED, -4, 4, 5.0);

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
            newLeftTarget = hardware.motorEsquerda.getCurrentPosition() + (int) (leftInches * Constants.COUNTS_PER_INCH);
            newRightTarget = hardware.motorDireita.getCurrentPosition() + (int) (rightInches * Constants.COUNTS_PER_INCH);

            // Seta o setPoint para nossos motores
            hardware.motorEsquerda.setTargetPosition(newLeftTarget);
            hardware.motorEsquerdaTras.setTargetPosition(newLeftTarget);
            hardware.motorDireita.setTargetPosition(newRightTarget);
            hardware.motorDireitaTras.setTargetPosition(newRightTarget);

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
                    (hardware.motorEsquerda.isBusy() && hardware.motorDireita.isBusy()) &&
                    (hardware.motorEsquerdaTras.isBusy() && hardware.motorDireitaTras.isBusy())) {

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

    // Método que printa a posição dos motores
    public void printPosicaoMotores() {
        telemetry.addData("Motores frontais", " em %7d :%7d",
                hardware.motorEsquerda.getCurrentPosition(), hardware.motorDireita.getCurrentPosition());
        telemetry.addData("Motores de trás", " em %7d :%7d",
                hardware.motorEsquerdaTras.getCurrentPosition(), hardware.motorDireitaTras.getCurrentPosition());
    }

    // Método que seta os modos para todos os motores
    public void setarModos(DcMotor.RunMode mode) {
        for (DcMotor motors : hardware.motores) {
            motors.setMode(mode);
        }
    }

    // Função que define a potência para todos motores
    public void setPowers(double speed) {
        for (DcMotor motors : hardware.motores) {
            motors.setPower(speed);
        }
    }
}