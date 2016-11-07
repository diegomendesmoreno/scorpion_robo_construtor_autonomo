Scorpion, o Robô construtor autônomo
==============
Projeto de um Robô Construtor Autônomo, como o trabalho de conclusão de curso de Engenharia Eletrônica de 2015.

Time
--------------
- Allan Dinato Martins
- André Ferro Kopelingh
- Diego Mendes Moreno

Links
--------------
- [Blog - Scorpion, o Robô construtor autônomo](http://engenoob.blogspot.com.br/2015/06/scorpion-robo-construtor-autonomo.html)
- [YouTube - Scorpion, o Robô construtor autônomo](https://youtu.be/ymfdhoq2V6s)


Transcrição do código Arduino scorpion_robo_construtor_autonomo.ino
--------------

```c
/* Nome   : scorpion_robo_construtor_autonomo
 * Alunos : Allan Dinato Martins
 *          Andre Ferro Kopelingh
 *          Diego Mendes Moreno
 */

/* Includes */
#include <Stepper.h>
#include <Ultrasonic.h>
#include <Servo.h>



/* Pinos */
//Steppers
#define D1           22   //azul
#define D2           24   //rosa
#define D3           26   //amarelo
#define D4           28   //laranja
#define E1           30   //azul
#define E2           32   //rosa
#define E3           34   //amarelo
#define E4           36   //laranja
//Servos
#define serv_base_1  50   //servo da base direito
#define serv_base_2  52   //servo da base esquerdo
#define serv_brac_1  51   //servo do braço direito
#define serv_brac_2  53   //servo do braço esquerdo
#define serv_garra   8    //servo da garra
//Sensores Ultrasom
///Ultra 10
#define echoPin_f    48
#define trigPin_f    49
///Ultra 4
#define echoPin_fc   35
#define trigPin_fc   37
///Ultra 9
#define echoPin_d1   46
#define trigPin_d1   47
///Ultra 8
#define echoPin_d2   44
#define trigPin_d2   45
///Ultra 7
#define echoPin_e1   42
#define trigPin_e1   43
///Ultra 6
#define echoPin_e2   40
#define trigPin_e2   41
///Ultra 5
#define echoPin_t    38
#define trigPin_t    39



/* Defines */
//Constantes
#define ATRASO         2
#define OFFSET_LATERAL 0.2
#define OFFSET_AJUSTE  0.5
#define DIVISOR_ARRUM  0.04
#define CNT_MAPA_MAX   4000



/* Subrotinas */
//Sensores Ultrasom
void leitura_sensores(void);
void ultrasom(void);
void ultra_valores(void);
void limpa_media(void);
//Steppers
void frente(float passos);
void tras(int passos);
void virar_dir(int graus);
void virar_esq(int graus);
//Braço
void inicial_servos();
void garra_abre();
void garra_fecha();
void base_avanca(int angbase);
void braco_avanca(int angbase, int angmeio);
void base_retorna(int angbase);
void braco_retorna(int angbase, int angmeio);
void braco_mix(int angbase, int angmeio);
void meio_avanca(int angmeio);
void meio_retorna(int angmeio);
void anda_reto(int passos);
void vai_base(int angbase);
void vai_braco(int angbase, int angmeio);
void pega(void);
void coloca_1st(void);
void coloca_2nd(void);
void coloca_3rd(void);



/* Variáveis Globais */
//Steppers
double passos_total = 2048;       //Nº de passos p/ 1 rev.
int angulo = 90;                  //Angulo de rotacao do eixo
double numero_de_passos;          //Nº de passos
int offset = 205;
//Ultrasom
volatile unsigned int cnt = 0;
volatile int flag_ultrasom = 1;
volatile float duration;
volatile float distancia_f[9];    //distancia[0] carrega a última média de valores
volatile float distancia_fc[9];   //distancia[0] carrega a última média de valores
volatile float distancia_d1[9];   //distancia[0] carrega a última média de valores
volatile float distancia_d2[9];   //distancia[0] carrega a última média de valores
volatile float distancia_e1[9];   //distancia[0] carrega a última média de valores
volatile float distancia_e2[9];   //distancia[0] carrega a última média de valores
volatile float distancia_t[9];    //distancia[0] carrega a última média de valores
volatile int i = 1;               //index dos valores de média
//Posicoes maximas dos motores
int posgarra = 30;
int posb_zero= 0;
int posb_fim= 180;
int posm_zero= 0;
int posm_fim= 180;
//contadores
int j;
int contb=0;
int contm=0;
// inicio base
int posb_dir=98;
int posb_esq=82;
//inicio meio
int posm_dir = 112;
int posm_esq = 43;
//velocidades
int delayb_avanco=40;
int delayb_retorno=40;
int delaym_avanco=40;
int delaym_retorno=50;
//pegar
int angb[5] = {90, 80, 70, 60, 0};
int angm[5] = {160, 130, 110, 90, 70};
//so enquanto testo e atualizo todo o programa
int angb_peg[5] = {90, 80, 70, 60, 50};
int angm_peg[5] = {150, 130, 110, 90, 70};
int angmhoje, angbhoje;
int d =500;
//Máquina de Estados
int ESTADO = 0;
volatile unsigned int cnt_mapa = 0;
float dist_alvo_d[6];
float dist_alvo_e[6];
int index_percurso = 0;               //index dos valores de distância de alvo
int flag_ajuste = 1;                  //flag ajuste
float andar;
float dist_andar;
float passos_step;
float dist_arrum;
int flag_arrum;
float const_arrum;



//Definição dos steppers
Stepper stepper_dir(passos_total, D1, D3, D2, D4);
Stepper stepper_esq(passos_total, E1, E3, E2, E4);
//Definição dos servos
Servo servob_dir;
Servo servob_esq;
Servo servo_garra;
Servo servom_dir;
Servo servom_esq;

//---------------------------------------------------------------------------------

void setup()
{
  /* Sensores Ultrasom */
  pinMode(trigPin_f, OUTPUT);
  pinMode(echoPin_f, INPUT);
  pinMode(trigPin_fc, OUTPUT);
  pinMode(echoPin_fc, INPUT);
  pinMode(trigPin_d1, OUTPUT);
  pinMode(echoPin_d1, INPUT);
  pinMode(trigPin_d2, OUTPUT);
  pinMode(echoPin_d2, INPUT);
  pinMode(trigPin_e1, OUTPUT);
  pinMode(echoPin_e1, INPUT);
  pinMode(trigPin_e2, OUTPUT);
  pinMode(echoPin_e2, INPUT);
  pinMode(trigPin_t, OUTPUT);
  pinMode(echoPin_t, INPUT);
  
  /* Configuração dos Steppers */
  stepper_dir.setSpeed(10);
  stepper_esq.setSpeed(10);
  
  //Servos
  servob_dir.attach(serv_base_1);
  servob_esq.attach(serv_base_2);
  servo_garra.attach(serv_garra);
  servom_dir.attach(serv_brac_1);
  servom_esq.attach(serv_brac_2);
  
  //seta o vetor de distancias
  limpa_media();
  
  //Distâncias da parede
  dist_alvo_d[0] = 6;
  dist_alvo_d[1] = 6.5;
  dist_alvo_d[2] = 16.2;
  dist_alvo_d[3] = 6.5;
  dist_alvo_d[4] = 30;
  dist_alvo_d[5] = 6.5;
  
  dist_alvo_e[0] = 30;
  dist_alvo_e[1] = 30;
  dist_alvo_e[2] = 15.5;
  dist_alvo_e[3] = 30;
  dist_alvo_e[4] = 6.5;
  dist_alvo_e[5] = 30;
  
  inicial_servos();
}

void loop()
{
  ++cnt;
  
  if(cnt >= ATRASO)
  {
    cnt = 0;
    leitura_sensores();
  }
  
  //Máquina de Estados
  switch(ESTADO)
  {
    case 0 :
      //localizar-se
      if(cnt_mapa < CNT_MAPA_MAX)
      {
        ++cnt_mapa;
      }
      else
      {
        vai_braco(130, 10);
        
        if(distancia_f[0] > 10)
        {
          ESTADO = 1;
        }
      }
      break;
      
    case 1 :
      //E8
      if(distancia_f[0] <= 10)
      {
        cnt_mapa = 0;
        ESTADO = 8;
        break;
      }
      
      if(index_percurso != 4)
      {
        //E2
        if(distancia_d1[0] > (distancia_d2[0] + OFFSET_LATERAL))
        {
          ESTADO = 2;
          break;
        }
        
        //E3
        if(distancia_d1[0] < (distancia_d2[0] - OFFSET_LATERAL))
        {
          ESTADO = 3;
          break;
        }
      }
      else if(index_percurso == 4)
      {
        //E2
        if(distancia_e1[0] < (distancia_e2[0] - OFFSET_LATERAL))
        {
          ESTADO = 2;
          break;
        }
        
        //E3
        if(distancia_e1[0] > (distancia_e2[0] + OFFSET_LATERAL))
        {
          ESTADO = 3;
          break;
        }
      }
      
      if(index_percurso != 4)
      {
        //E4 e E6
        if(distancia_d1[0] > (dist_alvo_d[index_percurso] + OFFSET_LATERAL))
        {
          if(flag_ajuste == 1)
          {
            ESTADO = 4;
            flag_ajuste = 0;
          }
          else
          {
            ESTADO = 6;
          }
          break;
        }
        
        //E5 e E7
        if(distancia_d1[0] < (dist_alvo_d[index_percurso] - OFFSET_LATERAL))
        {
          if(flag_ajuste == 1)
          {
            ESTADO = 5;
            flag_ajuste = 0;
          }
          else
          {
            ESTADO = 7;
          }
          break;
        }
      }
      
      if(index_percurso == 4)
      {
        //E4
        if(distancia_e1[0] < (dist_alvo_e[index_percurso] - OFFSET_LATERAL))
        {
          ESTADO = 6;
          break;
        }
      
        //E5
        if(distancia_e1[0] > (dist_alvo_e[index_percurso] + OFFSET_LATERAL))
        {
          if(flag_ajuste == 1)
          {
            ESTADO = 5;
            flag_ajuste = 0;
          }
          else
          {
            ESTADO = 7;
          }
          break;
        }
      }
      
      frente(1);
      break;
      
    case 2 :
      stepper_esq.step(-1);
      ESTADO = 1;
      break;
      
    case 3 :
      stepper_dir.step(1);
      ESTADO = 1;
      break;
    
    case 4 :
      //cálculo
      if(distancia_d1[0] > dist_alvo_d[index_percurso])
      {
        andar = distancia_d1[0] - dist_alvo_d[index_percurso];
        dist_andar = andar/0.707;
        passos_step = (dist_andar/20.0)*2048.0;
        
        //andar
        virar_dir(45);
        frente(passos_step);
        virar_esq(45);
      }
      else if(distancia_d1[0] < dist_alvo_d[index_percurso])
      {
        andar = dist_alvo_d[index_percurso] - distancia_d1[0];
        dist_andar = andar/0.707;
        passos_step = (dist_andar/20.0)*2048.0;
        
        //andar
        virar_esq(45);
        frente(passos_step);
        virar_dir(45);
      }
      ESTADO = 1;
      break;
    
    case 5 :
      //cálculo
      if(distancia_e1[0] > dist_alvo_e[index_percurso])
      {
        andar = distancia_e1[0] - dist_alvo_e[index_percurso];
        dist_andar = andar/0.707;
        passos_step = (dist_andar/20.0)*2048;
        
        //andar
        virar_esq(45);
        frente(passos_step);
        virar_dir(45);
      }
      ESTADO = 1;
      break;
    
    case 6 :
      stepper_esq.step(-5);
      frente(5);
      ESTADO = 1;
      break;
    
    case 7 :
      stepper_dir.step(5);
      frente(5);
      ESTADO = 1;
      break;
    
    case 8 :
      //localizar-se
      if(cnt_mapa < CNT_MAPA_MAX)
      {
        ++cnt_mapa;
      }
      else
      {
        if(index_percurso != 4)
        {
          if(distancia_d1[0] > (distancia_d2[0] + OFFSET_AJUSTE))
          {
            dist_arrum = distancia_d1[0] - distancia_d2[0];
            flag_arrum = 0;
            const_arrum = dist_arrum/DIVISOR_ARRUM;
          }
          else if(distancia_d1[0] < (distancia_d2[0] - OFFSET_AJUSTE))
          {
            dist_arrum = distancia_d2[0] - distancia_d1[0];
            flag_arrum = 1;
            const_arrum = dist_arrum/DIVISOR_ARRUM;
          }
        }
        else if(index_percurso == 4)
        {
          if(distancia_e1[0] > (distancia_e2[0] + OFFSET_AJUSTE))
          {
            dist_arrum = distancia_e1[0] - distancia_e2[0];
            flag_arrum = 2;
            const_arrum = dist_arrum/DIVISOR_ARRUM;
          }
          else if(distancia_e1[0] < (distancia_e2[0] - OFFSET_AJUSTE))
          {
            dist_arrum = distancia_e2[0] - distancia_e1[0];
            flag_arrum = 3;
            const_arrum = dist_arrum/DIVISOR_ARRUM;
          }
        }
        ESTADO = 9;
      }
      break;
    
    case 9 :
      while(const_arrum > 0)
      {
        if(flag_arrum == 0 || flag_arrum == 2)
        {
          stepper_dir.step(-1);
          stepper_esq.step(-1);
        }
        else if(flag_arrum == 1 || flag_arrum == 3)
        {
          stepper_dir.step(1);
          stepper_esq.step(1);
        }
        --const_arrum;
      }
      ESTADO = 10;
      break;
    
    case 10 :
      if(index_percurso == 0)
      {
        pega();
        
        tras(100);
        virar_esq(90);
        virar_esq(90);
        
        ++index_percurso;
        flag_ajuste = 1;
        cnt_mapa = 0;
        ESTADO = 0;
      }
      else if(index_percurso == 1)
      {
        coloca_1st();
        
        tras(200);
        virar_esq(90);
        virar_esq(90);
        
        ++index_percurso;
        flag_ajuste = 1;
        cnt_mapa = 0;
        ESTADO = 0;
      }
      else if(index_percurso == 2)
      {
        pega();
        
        tras(200);
        virar_esq(90);
        virar_esq(90);
        
        ++index_percurso;
        flag_ajuste = 1;
        cnt_mapa = 0;
        ESTADO = 0;
      }
      else if(index_percurso == 3)
      {
        coloca_2nd();
        
        tras(200);
        virar_esq(90);
        virar_esq(90);
        
        ++index_percurso;
        flag_ajuste = 1;
        cnt_mapa = 0;
        ESTADO = 0;
      }
      else if(index_percurso == 4)
      {
        pega();
        
        tras(100);
        virar_dir(90);
        virar_dir(90);
        
        ++index_percurso;
        flag_ajuste = 1;
        cnt_mapa = 0;
        ESTADO = 0;
      }
      else if(index_percurso == 5)
      {
        coloca_3rd();
        
        tras(300);
        
        //Posição escorpião
        vai_braco(130, 10);
        delay(d);
        
        garra_fecha();
        garra_abre();
        
        delay(2000);
        
        virar_esq(90);
        virar_esq(90);
        
        delay(5000);
        
        vai_braco(100, 110);
        
        while(1);
      }
      break;
  }
}

//---------------------------------------------------------------------------------

/* Subrotina : leitura_sensores
 */
void leitura_sensores(void)
{
  if(flag_ultrasom <= 5)
  {
    ultrasom();
    ++flag_ultrasom;
  }
  else
  {
    if(i <= 8)  //quantidade de medições para a média
    {
      ++i;
    }
    else
    {
      i = 1;
    }
    flag_ultrasom = 1;
    ultrasom();
  }
  
  ultra_valores();
}

/* Subrotina : ultrasom
 */
void ultrasom(void)
{
  switch(flag_ultrasom)
  {
    case 1 :
      //Ultrasom frente
      digitalWrite(trigPin_f, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin_f, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin_f, LOW);
      duration = pulseIn(echoPin_f, HIGH);
      distancia_f[i] = (340*duration/2)/10000;
      break;
    case 2 :
      //Ultrasom dir1
      digitalWrite(trigPin_d1, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin_d1, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin_d1, LOW);
      duration = pulseIn(echoPin_d1, HIGH);
      distancia_d1[i] = (340*duration/2)/10000;
      break;
    case 3 :
      //Ultrasom dir2
      digitalWrite(trigPin_d2, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin_d2, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin_d2, LOW);
      duration = pulseIn(echoPin_d2, HIGH);
      distancia_d2[i] = (340*duration/2)/10000;
      break;
    case 4 :
      //Ultrasom esq1
      digitalWrite(trigPin_e1, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin_e1, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin_e1, LOW);
      duration = pulseIn(echoPin_e1, HIGH);
      distancia_e1[i] = (340*duration/2)/10000;
      break;
    case 5 :
      //Ultrasom esq2
      digitalWrite(trigPin_e2, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin_e2, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin_e2, LOW);
      duration = pulseIn(echoPin_e2, HIGH);
      distancia_e2[i] = (340*duration/2)/10000;
      break;
  }
}

/* Subrotina : ultra_valores
 */
void ultra_valores(void)
{
  int j = 1;
  
  //Ultrasom frente
  for(j = 1;j <= 4; j++)
  {
    if(distancia_f[j] > 2 && distancia_f[j] < 100)
    {
      distancia_f[0] = (distancia_f[1]/8) + (distancia_f[2]/8) + (distancia_f[3]/8) + (distancia_f[4]/8) + (distancia_f[5]/8) + (distancia_f[6]/8) + (distancia_f[7]/8) + (distancia_f[8]/8);
    }
    else
    {
      distancia_f[j] = distancia_f[0];  //se o valor estiver fora, igualar a média
    }
  }
  
  //Ultrasom dir1
  for(j = 1;j <= 4; j++)
  {
    if(distancia_d1[j] > 2 && distancia_d1[j] < 100)
    {
      distancia_d1[0] = (distancia_d1[1]/8) + (distancia_d1[2]/8) + (distancia_d1[3]/8) + (distancia_d1[4]/8) + (distancia_d1[5]/8) + (distancia_d1[6]/8) + (distancia_d1[7]/8) + (distancia_d1[8]/8);
    }
    else
    {
      distancia_d1[j] = distancia_d1[0];  //se o valor estiver fora, igualar a média
    }
  }
  
  //Ultrasom dir2
  for(j = 1;j <= 4; j++)
  {
    if(distancia_d2[j] > 2 && distancia_d2[j] < 100)
    {
      distancia_d2[0] = (distancia_d2[1]/8) + (distancia_d2[2]/8) + (distancia_d2[3]/8) + (distancia_d2[4]/8) + (distancia_d2[5]/8) + (distancia_d2[6]/8) + (distancia_d2[7]/8) + (distancia_d2[8]/8);
    }
    else
    {
      distancia_d2[j] = distancia_d2[0];  //se o valor estiver fora, igualar a média
    }
  }
  
  //Ultrasom esq1
  for(j = 1;j <= 4; j++)
  {
    if(distancia_e1[j] > 2 && distancia_e1[j] < 100)
    {
      distancia_e1[0] = (distancia_e1[1]/8) + (distancia_e1[2]/8) + (distancia_e1[3]/8) + (distancia_e1[4]/8) + (distancia_e1[5]/8) + (distancia_e1[6]/8) + (distancia_e1[7]/8) + (distancia_e1[8]/8);
    }
    else
    {
      distancia_e1[j] = distancia_e1[0];  //se o valor estiver fora, igualar a média
    }
  }
  
  //Ultrasom esq2
  for(j = 1;j <= 4; j++)
  {
    if(distancia_e2[j] > 2 && distancia_e2[j] < 100)
    {
      distancia_e2[0] = (distancia_e2[1]/8) + (distancia_e2[2]/8) + (distancia_e2[3]/8) + (distancia_e2[4]/8) + (distancia_e2[5]/8) + (distancia_e2[6]/8) + (distancia_e2[7]/8) + (distancia_e2[8]/8);
    }
    else
    {
      distancia_e2[j] = distancia_e2[0];  //se o valor estiver fora, igualar a média
    }
  }
}

/* Subrotina : limpa_media
 */
void limpa_media(void)
{
  //frente
  distancia_f[0] = 11;
  distancia_f[1] = 11;
  distancia_f[2] = 11;
  distancia_f[3] = 11;
  distancia_f[4] = 11;
  distancia_f[5] = 11;
  distancia_f[6] = 11;
  distancia_f[7] = 11;
  distancia_f[8] = 11;
}

/* Subrotina : frente
 */
void frente(float passos)
{
  int i = 0;
  
  for(i = 0;i < passos;i++)
  {
    stepper_dir.step(1);
    stepper_esq.step(-1);
  }
}

/* Subrotina : tras
 */
void tras(int passos)
{
  int i = 0;
  
  for(i = 0;i < passos;i++)
  {
    stepper_dir.step(-1);
    stepper_esq.step(1);
  }
}

/* Subrotina : virar_dir
 */
void virar_dir(int graus)
{
  int i = 0;
  
  //Quantidade de passos, baseado no angulo determinado
  numero_de_passos = offset + 2*(graus / (360 / passos_total));
  
  for(i = 0;i < numero_de_passos;i++)
  {
    stepper_dir.step(-1);
    stepper_esq.step(-1);
  }
}

/* Subrotina : virar_esq
 */
void virar_esq(int graus)
{
  int i = 0;
  
  //Quantidade de passos, baseado no angulo determinado
  numero_de_passos = offset + 2*(graus / (360 / passos_total));
  
  for(i = 0;i < numero_de_passos;i++)
  {
    stepper_dir.step(1);
    stepper_esq.step(1);
  }
}

//-------------------------------------------------------------------------

/* Subrotina : inicial_servos
 */
 void inicial_servos()
 {
   servob_dir.write(posb_dir);
   servob_esq.write(posb_esq);
   servom_esq.write(posm_esq);
   servom_dir.write(posm_dir);
   servo_garra.write(posgarra); 
  
}

/* Subrotina : garra_abre
 */
void garra_abre()
{
  while(posgarra < 160)
  {
    servo_garra.write(posgarra); 
    ++posgarra;
    delay(10);
  }
  
}

/* Subrotina : garra_fecha
 */
void garra_fecha()
{
  while(posgarra > 40)
  {
    servo_garra.write(posgarra);
    --posgarra;
    delay(10);  
  } 
}

/* Subrotina : base_avanca
 */
void base_avanca(int angbase)
{
 while(posb_dir > angbase)
  {
    servob_dir.write(posb_dir);
    servob_esq.write(posb_esq);
    --posb_dir;
    ++posb_esq;
    delay(delayb_avanco);
  }
}

/* Subrotina : braco_avanca
 */
void braco_avanca(int angbase, int angmeio)
{
  while(posb_dir > angbase)
  {
    servob_dir.write(posb_dir);
    servob_esq.write(posb_esq);
    
    --posb_dir;
    ++posb_esq;
    delay(delaym_avanco);
    
    if(posm_dir< angmeio)
    {
      servom_esq.write(posm_esq);
      servom_dir.write(posm_dir); 
      ++posm_dir;
      --posm_esq;
    }
  }
  
  while(posm_dir < angmeio)
  {
    servom_esq.write(posm_esq);
    servom_dir.write(posm_dir); 
    ++posm_dir;
    --posm_esq;
    delay(delaym_avanco);
  }
  
}

/* Subrotina : base_retorna
 */
void base_retorna(int angbase)
{
  while(posb_dir < angbase)
  {
    servob_dir.write(posb_dir);
    servob_esq.write(posb_esq);
    ++posb_dir;
    --posb_esq;
    delay(delayb_retorno);
  }
  
}

/* Subrotina : braco_retorna
 */
void braco_retorna(int angbase, int angmeio)
{
  while(posb_dir < angbase)
  {
    servob_dir.write(posb_dir);
    servob_esq.write(posb_esq);
    ++posb_dir;
    --posb_esq;
    delay(delaym_retorno);
    
    if(posm_dir > angmeio)
    {
      servom_esq.write(posm_esq);
      servom_dir.write(posm_dir);
      --posm_dir;
      ++posm_esq;
    }
  
  }
  
  while(posm_dir > angmeio)
  {
    servom_esq.write(posm_esq);
    servom_dir.write(posm_dir);
    --posm_dir;
    ++posm_esq;
    delay(delaym_retorno);  
  }
  
}

/* Subrotina : braco_mix
 */
void braco_mix(int angbase, int angmeio)
{
  while(posb_dir > angbase)
  {
    servob_dir.write(posb_dir);
    servob_esq.write(posb_esq);
    
    --posb_dir;
    ++posb_esq;
    delay(delaym_avanco);
    
    if(posm_dir< angmeio)
    {
      servom_esq.write(posm_esq);
      servom_dir.write(posm_dir); 
      ++posm_dir;
      --posm_esq;
    }
    if(posm_dir > angmeio)
    {
      servom_esq.write(posm_esq);
      servom_dir.write(posm_dir);
      --posm_dir;
      ++posm_esq;
    } 
  }
  
  while(posb_dir < angbase)
  {
    servob_dir.write(posb_dir);
    servob_esq.write(posb_esq);
    ++posb_dir;
    --posb_esq;
    delay(delaym_retorno);
    
    if(posm_dir > angmeio)
    {
      servom_esq.write(posm_esq);
      servom_dir.write(posm_dir);
      --posm_dir;
      ++posm_esq;
    }
    if( posm_dir < angmeio)
    {
      servom_esq.write(posm_esq);
      servom_dir.write(posm_dir); 
      ++posm_dir;
      --posm_esq;
    }
  }
  
  //completa movimento meio
  while(posm_dir > angmeio)
  {
    servom_esq.write(posm_esq);
    servom_dir.write(posm_dir);
    --posm_dir;
    ++posm_esq;
    delay(delaym_retorno);  
  }
  //completa movimento meio
  while(posm_dir < angmeio)
  {
    servom_esq.write(posm_esq);
    servom_dir.write(posm_dir); 
    ++posm_dir;
    --posm_esq;
    delay(delaym_avanco);
  }
  
}
  
/* Subrotina : meio_avanca
 */
void meio_avanca(int angmeio)
{
  while(posm_dir < angmeio)
  {
    servom_esq.write(posm_esq);
    servom_dir.write(posm_dir); 
    ++posm_dir;
    --posm_esq;
    delay(delaym_avanco);
  }
}

/* Subrotina : meio_retorna
 */
void meio_retorna(int angmeio)
{
  while(posm_dir > angmeio)
  {
    servom_esq.write(posm_esq);
    servom_dir.write(posm_dir);
    --posm_dir;
    ++posm_esq;
    delay(delaym_retorno);  
  }
  
}

/* Subrotina : anda_reto
 */
void anda_reto(int passos)
{
  int angbase = posb_dir;
  int angmeio = posm_dir;
  j=passos;
  while(j>0){
  
  angbase= angbase-1;
  angmeio= angmeio-2;  
    
  base_avanca(angbase);
  meio_retorna(angmeio);
  delay(10);
 
  j--;
    }
}

/* Subrotina : vai_base
 * Descrição : 
 */
void vai_base(int angbase)
{
  if(angbase < posb_dir){
    base_avanca(angbase);
  }
  if(angbase > posb_dir){
    base_retorna(angbase);}  
}

/* Subrotina : vai_braco
 */
void vai_braco(int angbase, int angmeio)
{
  if(posb_dir > angbase && posm_dir < angmeio)
  braco_avanca(angbase, angmeio);
  
 if( posb_dir < angbase && posm_dir > angmeio)
   braco_retorna(angbase,angmeio);
 
 else
   braco_mix(angbase, angmeio);
}

/* Subrotina : pega
 */
void pega(void)
{
  delay(100);
  
  garra_abre();
  vai_braco(45, 14);
  delay(d);
  
  if(index_percurso == 0)
  {
    frente(300);
  }
  else if(index_percurso == 3)
  {
    frente(220);
  }
  else
  {
    frente(270);
  }  
  
  vai_braco(45, 14);
  delay(d);
  garra_fecha();
  vai_braco(50, 14);
  delay(d);
  tras(400);
  vai_braco(130, 10);
  delay(d);
}

/* Subrotina : coloca_1st
 */
void coloca_1st(void)
{
  //Anda até o bloquinho
  delay(200);
  tras(1000);
  delay(100);
  
  vai_braco(44, 14);
  delay(d);
  garra_abre();
  vai_braco(130, 10);
  delay(d);
}

/* Subrotina : coloca_2nd
 */
void coloca_2nd(void)
{
  //Anda até o bloquinho
  delay(200);
  frente(200);
  delay(100);
  
  vai_braco(55, 10);
  delay(d);
  garra_abre();
  vai_braco(130, 10);
  delay(d);
}

/* Subrotina : coloca_3rd
 */
void coloca_3rd(void)
{
  //Anda até o bloquinho
  delay(200);
  frente(220);
  delay(100);
  
  vai_braco(66, 6);
  delay(d);
  garra_abre();
  vai_braco(90, 50);
  delay(d);
}
```
