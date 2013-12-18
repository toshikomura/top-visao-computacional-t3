/*

unsigned long     millis();

int        get_ligth_sensor();

void set_sonar_angle(unsigned int theta);

void set_left_wheel_speed(unsigned int direction, unsigned char speed);

void set_right_wheel_speed(unsigned int direction, unsigned char speed);

*/

#include<stdio.h>

#include <Servo.h>

#define trigPin 4       //pino da emissao do pulso do ultrassom

#define echoPin 13      //pino da recepcao do pulso do ultrassom

#define servoPin 9      // pino pwm do servo

#define motor1Pin1 12   // pino 1 do motor da roda direita

#define motor1Pin2 11   // pino 2 do motor da roda direita

#define enablePin1 6    // pino pwm do motor da roda direita

#define motor2Pin1 7    // pino 1 do motor da roda esquerda

#define motor2Pin2 8    // pino 2 do motor da roda esquerda

#define enablePin2 5    // pino pwm do motor da roda esquerda

#define lightSensorPin 0 // pino para o sensor de luz;

Servo servo1;
float grid[10][10];// armazena as probabilidades de Marcov
int distparede_frente[10][10];// em relacao ao robo
int distparede_tras[10][10];
int distparede_esquerda[10][10];
int distparede_direita[10][10];
int checkpoints_x[40];
int checkpoints_y[40];
int uso_checkpoints[40];
int valores_i[10];//guarda os indices i e j do grid onde o robo pode estar pelo modelo do movimento
int valores_j[10];
int cont = 1;// conta o numero de posicoes do grid onde o robo pode estar para calcular a probabilidade uniforme dessas posicoes
int cont2, l, c;// cont2 usado para a normalizacao da matriz de probabilidades 
//int distance;
int distance = 0;

int virou_robo = 0;
int eh_check;
int i = 9;
int j = 0;
int k = 0;
int somador_prob, normalizacao;// servem para a normalizacao da matriz de probabilidades
int inic_matrizes = 1;// controlador para inicializar as matrizes de distancias do sonar apenas um vez durante todo o loop

int muda_i_ou_j = -1;//muda 'i'
int incrementa = -1;// servem para a movimentacao do robo(saber para onde ele esta indo

unsigned long ini;
void set_right_wheel_speed(unsigned int direction, char speed);
void set_left_wheel_speed(unsigned int direction, char speed);
void set_sonar_angle(unsigned int theta);
int distanceSonar();
int  get_light_sensor();



void setup() {

  pinMode(trigPin, OUTPUT);

  pinMode(echoPin, INPUT);

  pinMode(servoPin, OUTPUT);

  pinMode(motor1Pin1, OUTPUT);

  pinMode(motor1Pin2, OUTPUT);

  pinMode(enablePin1, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);

  pinMode(motor2Pin2, OUTPUT);

  pinMode(enablePin2, OUTPUT);

  servo1.attach(servoPin);

  servo1.write(0);

}



void inicializa_matrizes_dist() {
  int i, j, k, l;
  for(j=0;j<4;j++)
  {
   k = 0;
   for(i=0;i<10;i++)
   {
    distparede_frente[i][j] = k;
    k = k + 20;
   }
  }
  for(j=4;j<6;j++)
  {
    k = 0;
    for(i=0;i<8;i++)
    {
     distparede_frente[i][j] = k;
     k = k + 20;
    }
    distparede_frente[8][j] = 0;
    distparede_frente[9][j] = 20;
  }
  for(j=6;j<8;j++)
  {
    for(i=0;i<10;i++)
    {
      if(i % 2 == 0)
       distparede_frente[i][j] = 0;
      else
       distparede_frente[i][j] = 20;
    }
  }
  for(j=8;j<10;j++)
  {
    distparede_frente[0][j] = 0;
    distparede_frente[1][j] = 20;
    distparede_frente[2][j] = 0;
    distparede_frente[3][j] = 20;
    k = 40;
    for(i=4;i<10;i++)
    {
     distparede_frente[i][j] = k;
     k = k + 20;
    }
  }
  // TRAS
  for(j=0;j<4;j++)
  {
   k = 180;
   for(i=0;i<10;i++)
   {
    distparede_tras[i][j] = k;
    k = k - 20;
   }
  }
  for(j=4;j<6;j++)
  {
    k = 140;
    for(i=0;i<8;i++)
    {
     distparede_tras[i][j] = k;
     k = k - 20;
    }
    distparede_tras[8][j] = 20;
    distparede_tras[9][j] = 0;
  }
  for(j=6;j<8;j++)
  {
    for(i=0;i<10;i++)
    {
      if(i % 2 == 0)
       distparede_tras[i][j] = 20;
      else
       distparede_tras[i][j] = 0;
    }
  }
  for(j=8;j<10;j++)
  {
    distparede_tras[0][j] = 20;
    distparede_tras[1][j] = 0;
    k = 140;
    for(i=2;i<10;i++)
    {
     distparede_tras[i][j] = k;
     k = k - 20;
    }
  }
  // ESQUERDA
  k = 0;
  for(j=0;j<2;j++)
  {
    for(i=0;i<10;i++)
     distparede_esquerda[i][j] = k;
    k = 20;
  }
  k = 40;
  l = 0;
  for(j=2;j<4;j++)
  {
    distparede_esquerda[0][j] = k;
    distparede_esquerda[1][j] = k;
    for(i=2;i<10;i++)
     distparede_esquerda[i][j] = l;
    k = 60;
    l = 20;
  }
  k = 0;
  l = 40;
  for(j=4;j<6;j++)
  {
    for(i=0;i<6;i++)
     distparede_esquerda[i][j] = k;
    for(i=6;i<10;i++)
     distparede_esquerda[i][j] = l;
    k = 20;
    l = 60;
  }
  k = 40;
  for(j=6;j<8;j++)
  {
    for(i=0;i<6;i++)
     distparede_esquerda[i][j] = k;
    k = 60;
  }
  k = 80;
  for(j=6;j<8;j++)
  {
    for(i=6;i<8;i++)
     distparede_esquerda[i][j] = k;
    k = 100;
  }
  k = 0;
  for(j=6;j<8;j++)
  {
    for(i=8;i<10;i++)
     distparede_esquerda[i][j] = k;
    k = 20;
  }
  k = 80;
  for(j=8;j<10;j++)
  {
    for(i=0;i<4;i++)
     distparede_esquerda[i][j] = k;
    k = 100;
  }
  k = 0;
  for(j=8;j<10;j++)
  {
    for(i=4;i<8;i++)
     distparede_esquerda[i][j] = k;
    k = 20;
  }
  k = 40;
  for(j=8;j<10;j++)
  {
    for(i=8;i<10;i++)
     distparede_esquerda[i][j] = k;
    k = 60;
  }
  k = 60;
  l = 20;
  for(j=0;j<2;j++)
  {
    distparede_direita[0][j] = k;
    distparede_direita[1][j] = k;
    for(i=2;i<10;i++)
     distparede_direita[i][j] = l;
    k = 40;
    l = 0;
  }
  k = 20;
  for(j=2;j<4;j++)
  {
    for(i=0;i<6;i++)
     distparede_direita[i][j] = k;
    k = k - 20;
  }
  k = 100;
  for(j=2;j<4;j++)
  {
    for(i=6;i<8;i++)
     distparede_direita[i][j] = k;
    k = k - 20;
  }
  k = 60;
  for(j=2;j<4;j++)
  {
    for(i=8;i<10;i++)
     distparede_direita[i][j] = k;
    k = k - 20;
  }
  k = 100;
  for(j=4;j<6;j++)
  {
    for(i=0;i<4;i++)
     distparede_direita[i][j] = k;
    k = k - 20;
  }
  k = 60;
  for(j=4;j<6;j++)
  {
    for(i=4;i<8;i++)
     distparede_direita[i][j] = k;
    k = k - 20;
  }
  k = 20;
  for(j=4;j<6;j++)
  {
    for(i=8;i<10;i++)
     distparede_direita[i][j] = k;
    k = k - 20;
  }
  k = 60;
  for(j=6;j<8;j++)
  {
    for(i=0;i<4;i++)
     distparede_direita[i][j] = k;
    k = k - 20;
  }
  k = 20;
  for(j=6;j<8;j++)
  {
    for(i=4;i<8;i++)
     distparede_direita[i][j] = k;
    k = k - 20;
  }
  k = 60;
  for(j=6;j<8;j++)
  {
    for(i=8;i<10;i++)
     distparede_direita[i][j] = k;
    k = k - 20;
  }
  k = 20;
  for(j=8;j<10;j++)
  {
    for(i=0;i<10;i++)
     distparede_direita[i][j] = k;
    k = k - 20;
  }
  k = 100;

}

int checkpoints[10][10];

void inicializa_checkpoints(){
  int x, y, l;

  // zera matriz
  for (x=0; x<10; x++){
    for (y=0; y<10; y++){
      checkpoints[x][y] = 0;
    }
  }

  for(l=1; l<10; l++) {
    checkpoints[l][0] = 1;
  }

  checkpoints[0][1] = 1;

  for (l=1; l<8; l++){
    checkpoints[l][2] = 1;
  }

  checkpoints[8][3] = 2;

  for(l=4; l<7; l++){
    checkpoints[7][l] = 1;
  }

  for(l=4; l<7; l++){
    checkpoints[l][6] = 1;
  }

  checkpoints[3][5] = 1;
  checkpoints[2][5] = 2;

  for(l=6; l<9; l++){
    checkpoints[1][l] = 1;
  }

  for(l=5; l<9; l++){
    checkpoints[0][l] = 1;
  }

  checkpoints[1][4] = 1;

  for(l=6; l<8; l++){
    checkpoints[2][l] = 1;
  }

  for(l=3; l<8; l++){
    checkpoints[l][8] = 1;
  }

  for(l=8; l<10; l++){
    checkpoints[l][7] = 1;
  }

  checkpoints[9][8] = 1;

  for (x=0; x<10; x++){
    for (y=0; y<10; y++){
      printf ("%d ", checkpoints[x][y]);
    }
    printf ("\n");
  }
}


int verifica_checkpoint(int i,int j){
  printf ("Iniciou verifica_checkpoint\n");
  int l, valor_anterior;

  // Se eh checkpoint
  if (checkpoints[i][j] > 0){
    checkpoints[i][j]--;
    return(1);
  }
  return(0);
}



void loop() {
//void main () {

  Serial.begin(9600);

  // Testar light sensor
/*  if (get_light_sensor())
  {
     set_right_wheel_speed(1,0);
     set_left_wheel_speed(1,0);
     printf ("find light");
  }
*/

  if(inic_matrizes == 1)
  {
   inicializa_matrizes_dist();
   inicializa_checkpoints();
   inic_matrizes = 0;
  }
  //set_sonar_angle(90); // direciona o sonar para frente
  //ini = millis();
  cont2 = 0;
  virou_robo = 0;
  //distance = distanceSonar();
  //Serial.println(i);
  //Serial.println(j);
  valores_i[cont-1] = i;
  valores_j[cont-1] = j;
  //Serial.println("antes do k");

  //printf (" ================ Gride i %d j %d count %d\n", i, j, cont);
  for(k=0;k<cont;k++){// distribui as probabilidades de acordo com o modelo de movimento do robo e probabilidade uniforme
    //printf ("valor i %d valor j %d coloca %.2f\n", valores_i[k], valores_j[k], (float)1/cont);
    grid[valores_i[k]][valores_j[k]] = (float)1/cont;
   }

   /*for (l=0; l<10; l++){
     for (c=0; c<10; c++){
        printf ("%.2f ", grid[l][c]);
     }
     printf ("\n");
   }*/

   //Serial.println(" depois do k");
  //if(abs(distance) < 0)// nao achou parede a frente
  //{

      set_right_wheel_speed(1,0);
      set_left_wheel_speed(1,0);
      set_sonar_angle(-90);// vira o sonar para a direita
      distance = distanceSonar()/10;
      
      //printf ("Para robo, sonar para direita e pega valor do sonar %d\n", abs(distance));
      Serial.println("dist_dir = ");
      Serial.println(abs(distance));
      eh_check = verifica_checkpoint(i,j);
      //printf ("É check ? %d i %d j %d\n", eh_check, i, j);
      if(!eh_check && (abs(distance) > 25/* || abs(distance) == 0*/))// podemos nos movimentar para a direita ***incluir checkpoints para nao virar em algumas posicoes
      {
        //printf ("Virar para direita\n");
        //Serial.println("abababbaa");
       //gira 90 graus o robo para a direita
       ini = millis();
       while(millis()-ini < 870)
       //while(ini < 870)
       {
         set_right_wheel_speed(0,255);
         set_left_wheel_speed(1,255); // compensa uma diferenÃƒÂ§a Calibrar velocidade dessa roda provavelmente maior                                          // entre os motores
         //ini++;
       }
       //printf ("Robo faz curva para direita\n");
       //ini = 0;
       ini = millis();

       while(millis()-ini < 1240)// anda um grid a frente
       //while(ini < 1140)
       {
          set_right_wheel_speed(1,255-10);
          set_left_wheel_speed(1,255); // compensa uma diferenÃƒÂ§a Calibrar velocidade dessa roda provavelmente maior                                          // entre os motores
          //ini++;
       }
       //printf ("Robo anda para frente\n");
       //ini = 0;
       ini = millis();
        while(millis()-ini < 1140)
        //while(ini < 1140)
        {
         set_right_wheel_speed(1,0);
         set_left_wheel_speed(1,0);
         //ini++;
        }
        //printf ("Robo para\n");
        //ini = 0;
       virou_robo = 1;// Caso o robo vire nao podemos disparar o sonar para frente
       set_sonar_angle(0);// faz o sonar voltar a ficar alinhado com o robo
       distance = distanceSonar()/10;
       //distance = 30;
       if(abs(distance) > 25 && abs(distance) < 70)// podemos ter uma ideia mais concreta de onde o robo esta pelo sonar+modelo de movimento
       {
        if(muda_i_ou_j == -1 && incrementa == 1)// se incrementa o i
        {
         //printf ("estava incrementando i\n");
         for(l=0;l<10;l++)
          for(c=0;c<10;c++)
           if( distparede_esquerda[l][c] > abs(distance)-10 && distparede_esquerda[l][c] < abs(distance)+10)
            cont2++;
         for(l=0;l<10;l++)
          for(c=0;c<10;c++)
           if( distparede_esquerda[l][c] > abs(distance)-10 && distparede_esquerda[l][c] < abs(distance)+10)
            grid[l][c] = ((float)1/cont2) * grid[l][c];// combina a probabilidade do modelo de movimento com a do sonar
        }
        else
        if(muda_i_ou_j == -1 && incrementa == -1)
        {
         //printf ("estava decrementando i\n");
         for(l=0;l<10;l++)
          for(c=0;c<10;c++)
           if( distparede_direita[l][c] > abs(distance)-10 && distparede_direita[l][c] < abs(distance)+10)
            cont2++;
         for(l=0;l<10;l++)
          for(c=0;c<10;c++)
           if( distparede_direita[l][c] > abs(distance)-10 && distparede_direita[l][c] < abs(distance)+10)
            grid[l][c] = ((float)1/cont2) * grid[l][c];
         }
         else
         if(muda_i_ou_j == 1 && incrementa == 1)
         {

          //printf ("estava incrementando j\n");
          for(l=0;l<10;l++)
           for(c=0;c<10;c++)
            if( distparede_tras[l][c] > abs(distance)-10 && distparede_tras[l][c] < abs(distance)+10)
             cont2++;
          for(l=0;l<10;l++)
           for(c=0;c<10;c++)
            if( distparede_tras[l][c] > abs(distance)-10 && distparede_tras[l][c] < abs(distance)+10)
             grid[l][c] = ((float)1/cont2) * grid[l][c];
          }
          else
          if(muda_i_ou_j == 1 && incrementa == -1)
          {

           //printf ("estava decrementando j\n");
           for(l=0;l<10;l++)
            for(c=0;c<10;c++)
             if( distparede_frente[l][c] > abs(distance)-10 && distparede_frente[l][c] < abs(distance)+10)
              cont2++;
           for(l=0;l<10;l++)
            for(c=0;c<10;c++)
             if( distparede_frente[l][c] > abs(distance)-10 && distparede_frente[l][c] < abs(distance)+10)
              grid[l][c] = ((float)1/cont2) * grid[l][c];
           }
       }
       for(k=0;k<cont;k++)
        grid[valores_i[k]][valores_j[k]] = 0.0;// como o robo fez uma curva zera-se todas as probabilidades das posicoes fora da curva
       cont = 0;
       if(muda_i_ou_j == 1 && incrementa == 1)// verifica qual variavel vai ser alterada a partir do proximo movimento do robo
        {
         //printf ("estava incrementando j\n");
         muda_i_ou_j = -1;
         grid[i+1][j] = 1.0;
         incrementa = 1;
        }
        else
        if(muda_i_ou_j == -1 && incrementa == 1)
        {
         //printf ("estava incrementando i\n");
         muda_i_ou_j = 1;
         grid[i][j-1] = 1.0;
         incrementa = -1;
        }
        else
        if(muda_i_ou_j == 1 && incrementa == -1)
        {
         //printf ("estava decrementando j\n");
         muda_i_ou_j = -1;
         grid[i-1][j] = 1.0;
         incrementa = -1;
        }
        else
        if(muda_i_ou_j == -1 && incrementa == -1)// muda_i_ou_j == 1 => muda j
        {
         //printf ("estava decrementando i\n");
         muda_i_ou_j = 1;
         grid[i][j+1] = 1.0;
         incrementa = 1;
          //printf ("teste muda_i_ou_j %d inc %d\n", muda_i_ou_j, incrementa);
        }

        //printf ("muda_i_ou_j %d inc %d\n", muda_i_ou_j, incrementa);
      }
      else
      {
       set_right_wheel_speed(1,0);
       set_left_wheel_speed(1,0);
       set_sonar_angle(90);// vira o sonar para a esquerda
       distance = distanceSonar()/10;
       //distance = 30;

       //printf ("Para robo, sonar para esquerda e pega valor do sonar %d\n", abs(distance));
       //Serial.println("dist_esquerda = ");
       //Serial.println(abs(distance));
       //printf ("É check ? %d i %d j %d\n", eh_check, i, j);
       if(!eh_check && (abs(distance) > 25 || abs(distance) == 0)) // Se ele vai para a esquerda
       {

        //printf ("Virar para esquerda\n");
        //Serial.println("abababbaa");
        virou_robo = 1;// caso o robo vire, o sonar nao pode ser disparado para frente
        //gira 90 graus o robo para a esquerda
        ini = millis();
        while(millis()-ini < 870)
        //while(ini < 870)
        {
          set_right_wheel_speed(1,255);
          set_left_wheel_speed(0,255); // compensa uma diferenÃƒÂ§a Calibrar velocidade dessa roda provavelmente maior                                          // entre os motores
          //ini++;
        }
        //printf ("Robo faz curva para esquerda\n");
        //ini = 0;
        ini = millis();// anda um grid a frente
        while(millis()-ini < 870)
        //while(ini < 870)
        {
          set_right_wheel_speed(1,255-10);
          set_left_wheel_speed(1,255); // compensa uma diferenÃƒÂ§a Calibrar velocidade dessa roda provavelmente maior                                          // entre os motores
          //ini++;
        }
        //printf ("Robo anda para frente\n");
        //ini = 0;

        ini = millis();
        while(millis()-ini < 1140)
        //while(ini < 1140)
        {
         set_right_wheel_speed(1,0);
         set_left_wheel_speed(1,0);
         //ini++;
        }
        //printf ("Robo para\n");
        //ini = 0;
        set_sonar_angle(0);// faz o sonar voltar a ficar alinhado com o robo
        distance = distanceSonar()/10;
        //distance = 30;
        if(abs(distance) > 25 && abs(distance) < 70)// podemos ter uma ideia mais concreta de onde o robo esta pelo sonar+modelo de movimento
        {
         if(muda_i_ou_j == -1 && incrementa == 1)
         {
          for(l=0;l<10;l++)
           for(c=0;c<10;c++)
            if( distparede_direita[l][c] > abs(distance)-10 && distparede_direita[l][c] < abs(distance)+10)
             cont2++;
          for(l=0;l<10;l++)
           for(c=0;c<10;c++)
            if( distparede_direita[l][c] > abs(distance)-10 && distparede_direita[l][c] < abs(distance)+10)
             grid[l][c] = ((float)1/cont2) * grid[l][c];
         }
         else
         if(muda_i_ou_j == -1 && incrementa == -1)
         {
          for(l=0;l<10;l++)
           for(c=0;c<10;c++)
            if( distparede_esquerda[l][c] > abs(distance)-10 && distparede_esquerda[l][c] < abs(distance)+10)
             cont2++;
          for(l=0;l<10;l++)
           for(c=0;c<10;c++)
            if( distparede_esquerda[l][c] > abs(distance)-10 && distparede_esquerda[l][c] < abs(distance)+10)
             grid[l][c] = ((float)1/cont2) * grid[l][c];
          }
          else
          if(muda_i_ou_j == 1 && incrementa == 1)
          {
           for(l=0;l<10;l++)
            for(c=0;c<10;c++)
             if( distparede_frente[l][c] > abs(distance)-10 && distparede_frente[l][c] < abs(distance)+10)
              cont2++;
           for(l=0;l<10;l++)
            for(c=0;c<10;c++)
             if( distparede_frente[l][c] > abs(distance)-10 && distparede_frente[l][c] < abs(distance)+10)
              grid[l][c] = ((float)1/cont2) * grid[l][c];
           }
           else
           if(muda_i_ou_j == 1 && incrementa == -1)
           {
            for(l=0;l<10;l++)
             for(c=0;c<10;c++)
              if( distparede_tras[l][c] > abs(distance)-10 && distparede_tras[l][c] < abs(distance)+10)
               cont2++;
            for(l=0;l<10;l++)
             for(c=0;c<10;c++)
              if( distparede_tras[l][c] > abs(distance)-10 && distparede_tras[l][c] < abs(distance)+10)
               grid[l][c] = ((float)1/cont2) * grid[l][c];
            }
          }
          for(k=0;k<cont;k++)
           grid[valores_i[k]][valores_j[k]] = 0.0;
          cont = 0;
          if(muda_i_ou_j == 1 && incrementa == 1)
          {
           muda_i_ou_j = -1;
           grid[i-1][j] = 1.0;
           incrementa = -1;
          }
          else
          if(muda_i_ou_j == -1 && incrementa == 1)
          {
           muda_i_ou_j = 1;
           grid[i][j+1] = 1.0;
           incrementa = 1;
          }
          else
          if(muda_i_ou_j == 1 && incrementa == -1)
          {
           muda_i_ou_j = -1;
           grid[i+1][j] = 1.0;
           incrementa = 1;
          }
          else
          if(muda_i_ou_j == -1 && incrementa == -1)// muda_i_ou_j == 1 => muda j
          {
           muda_i_ou_j = 1;
           grid[i][j-1] = 1.0;
           incrementa = -1;
          }
         }

         //printf ("muda_i_ou_j %d inc %d\n", muda_i_ou_j, incrementa);
      }
      if(virou_robo == 0)// se o robo nao fez curva ele vai andar para frente e deve disparar o sonar
      {

        //printf ("Virar para frente\n");
        //Serial.println("distanciaaa");
        //set_right_wheel_speed(1,255-10);
        //set_left_wheel_speed(1,255);
        set_sonar_angle(0);
        distance = distanceSonar()/10;
        //distance = 30;
        //printf ("Robo envia sonar e recebe %d\n", abs(distance));
        ini = millis();
        while(millis()-ini < 1400)
        //while(ini < 870)
        {
          set_right_wheel_speed(1,255-10);
          set_left_wheel_speed(1,255); // compensa uma diferenÃƒÂ§a Calibrar velocidade dessa roda provavelmente maior                                          // entre os motores
          //ini++;
        }

        //printf ("Robo anda para frente\n");
        //ini = 0;
        ini = millis();
        set_right_wheel_speed(1,0);
        set_left_wheel_speed(1,0);
        //printf ("Robo para\n");

        //Serial.println("dist_reto = ");
        //Serial.println(abs(distance));
        if(abs(distance) > 25 && abs(distance) < 70)// podemos ter uma ideia mais concreta de onde o robo esta pelo sonar+modelo de movimento
        {
         if(muda_i_ou_j == -1 && incrementa == 1)
         {
          for(l=0;l<10;l++)
           for(c=0;c<10;c++)
            if( distparede_frente[l][c] > abs(distance)-10 && distparede_frente[l][c] < abs(distance)+10)
             cont2++;
          for(l=0;l<10;l++)
           for(c=0;c<10;c++)
            if( distparede_frente[l][c] > abs(distance)-10 && distparede_frente[l][c] < abs(distance)+10)
             grid[l][c] = ((float)1/cont2) * grid[l][c];
         }
         else
         if(muda_i_ou_j == -1 && incrementa == -1)
         {
          for(l=0;l<10;l++)
           for(c=0;c<10;c++)
            if( distparede_tras[l][c] > abs(distance)-10 && distparede_tras[l][c] < abs(distance)+10)
             cont2++;
          for(l=0;l<10;l++)
           for(c=0;c<10;c++)
            if( distparede_tras[l][c] > abs(distance)-10 && distparede_tras[l][c] < abs(distance)+10)
             grid[l][c] = ((float)1/cont2) * grid[l][c];
          }
          else
          if(muda_i_ou_j == 1 && incrementa == 1)
          {
           for(l=0;l<10;l++)
            for(c=0;c<10;c++)
             if( distparede_direita[l][c] > abs(distance)-10 && distparede_direita[l][c] < abs(distance)+10)
              cont2++;
           for(l=0;l<10;l++)
            for(c=0;c<10;c++)
             if( distparede_direita[l][c] > abs(distance)-10 && distparede_direita[l][c] < abs(distance)+10)
              grid[l][c] = ((float)1/cont2) * grid[l][c];
           }
           else
           if(muda_i_ou_j == 1 && incrementa == -1)
           {
            for(l=0;l<10;l++)
             for(c=0;c<10;c++)
              if( distparede_esquerda[l][c] > abs(distance)-10 && distparede_esquerda[l][c] < abs(distance)+10)
               cont2++;
            for(l=0;l<10;l++)
             for(c=0;c<10;c++)
              if( distparede_esquerda[l][c] > abs(distance)-10 && distparede_esquerda[l][c] < abs(distance)+10)
               grid[l][c] = ((float)1/cont2) * grid[l][c];// combina o valor da probabilidade do sonar com o valor de onde a gente acha que o robo esta
            }
          }
      }
      //printf ("Normaliza matrizes\n");
      // NORMALIZA A MATRIZ DE PROBABILIDADES DE FORMA A FECHAR 100%
      //Serial.print("hahha");
      cont2 = 0;
      somador_prob = 0;
      for(l=0;l<10;l++)
       for(c=0;c<10;c++)
        if(grid[l][c] != 0)
        {
         cont2++;
         somador_prob = somador_prob + grid[l][c];
        }
      normalizacao = 1.0 - somador_prob;
      normalizacao = normalizacao / (float)cont2;
      for(l=0;l<10;l++)
       for(c=0;c<10;c++)
        if(grid[l][c] != 0)
         grid[l][c] = grid[l][c] + normalizacao;

      // Imprime matriz
      /*
      for(l=0;l<10;l++)
      {
        for(c=0;c<10;c++)
        {
         //Serial.print(grid[l][c]);
         //Serial.print(" ");
          printf ("%.2f ", grid[l][c]);
        }
        //Serial.println(" ");
        printf ("\n");
      }
      */

       //set_right_wheel_speed(1,0);
       //set_left_wheel_speed(1,0); // compensa uma diferenÃ§a Calibrar velocidade dessa roda provavelmente maior
                                      // entre os motores
       //Serial.println("77");
       if(muda_i_ou_j == 1)// muda j
       {
        // printf ("muda j\n");
         if(incrementa == 1){ printf ("incrementa\n");
          j++;
         }
         else {// printf ("decrementa\n");
          j--;
         }
       }
       else
       {
         //printf ("muda i\n");
         if(incrementa == 1){ printf ("incrementa\n");
          i++;
         }
         else
         { //printf ("decrementa\n");
          i--;
          //Serial.println(" p = 444444444444");
         }
       }
       cont++;
      //distance = distanceSonar();
      //distance = 30;
  //}
  /*else
  {
        while(millis()-ini < 2000) // Calibrar tempo para fazer volta de 90 graus
        {
          set_right_wheel_speed(1,255);
          set_left_wheel_speed(0,255-17);
        }
  }
  // para o robo
  set_right_wheel_speed(1,0);
  set_left_wheel_speed(1,0);*/

} // fim do while(1)



void set_right_wheel_speed(unsigned int direction, char speed)

{

        if (direction == 1)

        {

          digitalWrite(motor1Pin1, HIGH);

          digitalWrite(motor1Pin2, LOW);

          analogWrite(enablePin1, speed);

        }

        else

        {

          digitalWrite(motor1Pin1, LOW);

          digitalWrite(motor1Pin2, HIGH);

          analogWrite(enablePin1, speed);

        }

}


void set_left_wheel_speed(unsigned int direction, char speed)

{

        if (direction == 1)

        {

          digitalWrite(motor2Pin1, HIGH);

          digitalWrite(motor2Pin2, LOW);

          analogWrite(enablePin2, speed);

        }

        else

        {

          digitalWrite(motor2Pin1, LOW);

          digitalWrite(motor2Pin2, HIGH);

          analogWrite(enablePin2, speed);

        }

}


void set_sonar_angle(unsigned int theta)

{

   servo1.write(90+theta);

   delay(300);

}


int distanceSonar()

{

  long duration;

  digitalWrite(trigPin, LOW);

  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);

  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  return (int)((duration/2) / 2.91);

}


int  get_light_sensor()
{
   return analogRead(lightSensorPin);
}

