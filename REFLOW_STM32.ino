/*
*Exemplo de curva de temperatura para o processo de Reflow.
* Temperature                Magic Happens Here!
* 245-|                                               x  x  
*     |                                            x        x
*     |                                         x              x
*     |                                      x                    x
* 200-|                                   x                          x
*     |                              x    |                          |   x  
*     |                         x         |                          |       x
*     |                    x              |                          |
* 150-|               x                   |                          |
*     |             x |                   |                          |
*     |           x   |                   |                          |
*     |         x     |                   |                          |
*     |       x       |                   |                          |
*     |     x         |                   |                          |
*     |   x           |                   |                          |
* 30 -| x             |                   |                          |
*     |<  60 - 90 s  >|<    60 - 120 s   >|<       60 - 150 s       >|
*     | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
*  0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
*                                                                Time (Seconds)
*
* ==========================================
*/
//Inclusao das bibliotecas externas

#include <PID_v1.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>
#include <MAX6675_SPI_2.h>
#include <Wire.h>
#include <ARDUINO.h>
#include <EEPROM.h>

LiquidCrystal_I2C lcd(0x3f, 16, 2);  //Configuraçao do LCD
//configuraçao dos timers
TIM_TypeDef *Instance = TIM2;
TIM_TypeDef *Instance2 = TIM3;
HardwareTimer *MyTim = new HardwareTimer(Instance);
HardwareTimer *MyTim2 = new HardwareTimer(Instance2);

MAX6675 myMAX6675(PB12);                                                                        //Definiçao do pino CS do MAX6675
const uint8_t iconTemperature[8] PROGMEM = { 0x04, 0x0E, 0x0E, 0x0E, 0x0E, 0x1F, 0x1F, 0x0E };  //PROGMEM saves variable to flash & keeps dynamic memory free
#define LCD_DEGREE_SYMBOL 0xDF                                                                  //degree symbol from lcd ROM, see p.9 of GDM2004D datasheet
#define LCD_SPACE_SYMBOL 0x20                                                                   //degree symbol from lcd ROM, see p.9 of GDM2004D datasheet

const int triac_pin = PB1;          //Sinal de controle do Triac
const int buton_exit = PA9;         //Botao de ativaçao da saida
const int boton_enter = PA10;       //Botao de menu
const int led_01 = PB10;            //Sinal de acinamneto do led
const int led_02 = PB0;             //Sinal de acinamneto do led
const int Buzzer = PB11;            //Sinal de controle do Buzzer
const int zero_crossing_pin = PA8;  //Sinal do zero crossing
const int LM35 = A0;                //Sinal do zero crossing
const int cooling_pin = A7;         //Sinal do zero crossing


#define MAX_TEMPERATURE 45        //max temp, °C
#define pulso_triac 5			  //Duraçao do pulso de disparo do Triac
int valor_triac = 0;
bool zeroCross_signal;
int power_triac = 0;
bool output_power_on = false;
volatile int state = LOW;         // must declare volatile, since it's
float temperature = 0;            //
bool enable_power = false;


volatile int encoderPosition = 0;
volatile int factor = 1;         //Numero de encremento do encoder
volatile int encoderMax = 1000;  //Valor maximo para o encoder
volatile int encoderMin = 1;     //Valor minimo para o encoder
int *point_encoderPosition = 0;  //Ponteiro para controle do encorder.
int aux;

unsigned long lastPublishMillis;  //Temporizaçao.
int *tempo_segundos;
bool tempo_segundos_modo = true;  // true for increment/ false for decrement

int M0A = 0;         // Variave de controle dos menus nivel 0
int M1A = 0;         // Variave de controle dos menus nivel 1
int M2A = 0;         // Variave de controle dos menus nivel 2
int menu_level = 0;  // Variave de controle dos menus - nivel

int button_flag_enter;      // Variave de controle dos menus
bool button_confirm = false;// Variave de controle dos menus
bool button_exit = false;   // Variave de controle dos menus

int Preheat_Temp; //Varives contendo os parametros do processo.
int Preheat_Time; //Varives contendo os parametros do processo.
int Soaking_Temp; //Varives contendo os parametros do processo.
int Soaking_Time; //Varives contendo os parametros do processo.
int Reflow_Temp;  //Varives contendo os parametros do processo.
int Reflow_Time;  //Varives contendo os parametros do processo.
int cooling_Time; //Varives contendo os parametros do processo.
int cooling_Temp; //Varives contendo os parametros do processo.
int stufa_Time;   //Varives contendo os parametros do processo.
int stufa_Temp;   //Varives contendo os parametros do processo.
int pid_kp;       //Varives contendo os parametros do processo.
int pid_ki;       //Varives contendo os parametros do processo.
int pid_kd;       //Varives contendo os parametros do processo.

float consKp;     //Varives contendo os parametros do processo.
float consKi;     //Varives contendo os parametros do processo.
float consKd;     //Varives contendo os parametros do processo.
float aux_temp;   //Varives contendo os parametros do processo.

int total_time_process;  //variavel de tempo do processo
int time_process;        // varivel de controle de tempo atual do processo.

double Setpoint, Input, Output; //Varives contendo os parametros do processo.
PID myPID(&Input, &Output, &Setpoint, 2.0, 5.0, 1.0, DIRECT);

//Definiçao dos textos que serao mostrados no Display.
char *TxT_01 = (char *)"Preheat - Temp  ";
char *TxT_02 = (char *)"Preheat - Time  ";
char *TxT_03 = (char *)"Soaking - Temp  ";
char *TxT_04 = (char *)"Soaking - Time  ";
char *TxT_05 = (char *)"Reflow - Temp   ";
char *TxT_06 = (char *)"Reflow - Time   ";
char *TxT_07 = (char *)"Cooling - Temp  ";
char *TxT_08 = (char *)"Cooling - Time  ";
char *TxT_09 = (char *)"Stufa - Temp    ";
char *TxT_10 = (char *)"Stufa - Time    ";
char *TxT_11 = (char *)"PID - Kp        ";
char *TxT_12 = (char *)"PID - Ki        ";
char *TxT_13 = (char *)"PID - Kd        ";
char *TxT_14 = (char *)"Preheat  ";
char *TxT_15 = (char *)"Soaking  ";
char *TxT_16 = (char *)"Reflow   ";
char *TxT_17 = (char *)"Cooling  ";
char *TxT_18 = (char *)"Heating  ";
//Prototipo de algumas funçoes - para evitar alguns erros de copilaçao.
void print_temp(float temp, int posiçao, int line = 1);
void print_time(int time, int posiçao);
void print_time2(int time, int posiçao);
void print_PID(float pid_value, int posiçao);
//-----------------------------------------------------------------------------
void read_memory() {
	//Leitura dos dados armazenados na memoria flash do STM32
  Preheat_Temp = readEepromInt(0x00);  //
  Preheat_Time = readEepromInt(0x02);  //
  Soaking_Temp = readEepromInt(0x04);  //
  Soaking_Time = readEepromInt(0x06);  //
  Reflow_Temp = readEepromInt(0x08);   //
  Reflow_Time = readEepromInt(0x0A);   //
  cooling_Time = readEepromInt(0x0C);  //
  cooling_Temp = readEepromInt(0x0E);  //
  stufa_Temp = readEepromInt(0x10);    //
  stufa_Time = readEepromInt(0x12);    //
  pid_kp = readEepromInt(0x14);        //
  pid_ki = readEepromInt(0x16);        //
  pid_kd = readEepromInt(0x18);        //
}

void print_memory() {
	//Funçao de debug
  Serial.print("Preheat_Temp:");
  Serial.println(Preheat_Temp);
  Serial.print("Preheat_Time:");
  Serial.println(Preheat_Time);
  Serial.print("Soaking_Temp:");
  Serial.println(Soaking_Temp);
  Serial.print("Soaking_Time:");
  Serial.println(Soaking_Time);
  Serial.print("Reflow_Temp :");
  Serial.println(Reflow_Temp);
  Serial.print("Reflow_Time :");
  Serial.println(Reflow_Time);
  Serial.print("cooling_Time:");
  Serial.println(cooling_Time);
  Serial.print("cooling_Temp:");
  Serial.println(cooling_Temp);
  Serial.print("stufa_Temp  :");
  Serial.println(stufa_Temp);
  Serial.print("stufa_Time  :");
  Serial.println(stufa_Time);
}
void upd_pid_parametros() {
//Funçao para carregar os paramentos do controle PID no formarto correto.
  consKp = pid_kp / 1000;
  consKi = pid_ki / 1000;
  consKd = pid_kd / 1000;
  myPID.SetTunings(consKp, consKi, consKd);
}
//-----------------------------------------------------------------------------
//Funçao para Formatar a EEPROM
void eeprom_format() {
  for (int i = 0; i < 20; i++) {
    EEPROM.write(i, 0x00);
    delay(10);
  }
}
//-----------------------------------------------------------------------------
//Funçao para Imprimir o conteudo da EEPROM
void print_eeprom_memory() {
  for (int i = 0; i < 20; i++) {
    Serial.print("END:");
    Serial.print(i);
    Serial.print(" - ");
    Serial.println(EEPROM.read(i));
    delay(10);
  }
}
//-----------------------------------------------------------------------------
//Funçao para Salvar um parametro na EEPROM
void writeEepromInt(int value, int location) {
  EEPROM.write(location, value);
  EEPROM.write(location + 1, value >> 8);
}
//-----------------------------------------------------------------------------
//Funçao para Ler um parametro na EEPROM
int readEepromInt(int location) {
  int val;
  val = (EEPROM.read(location + 1) << 8);
  val |= EEPROM.read(location);
  return val;
}
//-----------------------------------------------------------------------------
//Funçao para tratar o sinal do botao ENTER
bool checa_bot_enter() {
  bool buttonState = digitalRead(boton_enter);
  if (buttonState == LOW && buttonState != button_flag_enter) {
    bip();
    button_flag_enter = buttonState;
    return true;
  }
  button_flag_enter = buttonState;
  return false;
}
//-----------------------------------------------------------------------------
//Funçao para tratar o sinal do botao EXIT
bool checa_bot_exit() {

  bool buttonState = digitalRead(buton_exit);
  if (buttonState == LOW && buttonState != button_flag_enter) {
    bip();
    button_flag_enter = buttonState;
    return true;
  }
  button_flag_enter = buttonState;
  return false;
}
//-----------------------------------------------------------------------------
//Funçao para tratar o sinal do botao ENTER quando presionado por um longo period de tempo.
bool checa_bot_confirm() {
  int static aux = 0;
  bool buttonState = digitalRead(boton_enter);
  if (buttonState == LOW) {
    aux++;
  } else {
    aux = 0;
    button_confirm = false;
  }
  if (aux > 10 && !button_confirm) {
    bip();
    bip();
    bip();
    bip();
    bip();
    button_confirm = true;
    return true;
  }
  return false;
}
//-----------------------------------------------------------------------------
//Funçao para tratar o sinal do botao ENTER - presionado momenteneamente ou por um longo periodo de tempo.
int checa_bot_confirm2() {
  int static aux = 0;
  bool buttonState = digitalRead(boton_enter);

  if (!buttonState && !button_confirm) {
    if (aux == 0) {
      bip();
    }
    aux++;
  }
  if (aux >= 5 && !button_confirm) {
    bip();
    bip();
    bip();
    bip();
    bip();
    button_confirm = true;
    return 2;
  }
  if (buttonState && aux > 0 && !button_confirm) {

    button_confirm = true;
    return 1;
  }
  if (buttonState) {
    aux = 0;
    button_confirm = false;
  }
  return 0;
}
//-----------------------------------------------------------------------------
//Funçao para configuraçao dos parametros de TEMPERATURA.
void confing_param_temp(char *texto_1, volatile int *encorder_value, int min_value, int *parametro, int pos_memory, int retorno) {
  lcd.setCursor(0, 0);
  lcd.print(texto_1);
  if (menu_level == 1) {
    if (checa_bot_enter()) {
      menu_level = 2;
      *encorder_value = *parametro;
      encoderMax = 450;
      factor = 1;
    }
    if (checa_bot_exit()) {
      lcd.setCursor(0, 0);
      lcd.print("                ");
      menu_level = 0;
      *encorder_value = 0;
      read_memory();
    }
  }
  if (menu_level == 2) {
    encoderMin = min_value;
    *parametro = *encorder_value;
    int valor = *parametro;
    print_temp(valor, 4);
    if (checa_bot_confirm()) {
      writeEepromInt(*parametro, pos_memory);
      lcd.setCursor(0, 1);
      lcd.print("                ");
      menu_level = 1;
      *encorder_value = retorno;
      encoderMin = 0;
    }
    if (checa_bot_exit()) {
      lcd.setCursor(0, 1);
      lcd.print("                ");
      menu_level = 1;
      *encorder_value = retorno;
      encoderMin = 0;
    }
  }
}
//-----------------------------------------------------------------------------
//Funçao para configuraçao dos parametros de TEMPO.
//confing_param_time(         TxT_10, &encoderPosition,                  &stufa_Time,     0x12,           9);
void confing_param_time(char *texto_1, volatile int *encorder_value, int *parametro, int pos_memory, int retorno) {
  lcd.setCursor(0, 0);
  lcd.print(texto_1);
  if (menu_level == 1) {
    if (checa_bot_enter()) {
      menu_level = 2;

      if (retorno == 9) {
        encoderMax = 18000;
        factor = 60;
      } else {
        encoderMax = 180;
        factor = 10;
      }

      encoderMin = 0;
      *encorder_value = *parametro;
    }
    if (checa_bot_exit()) {
      lcd.setCursor(0, 0);
      lcd.print("                ");
      menu_level = 0;
      *encorder_value = 0;
      factor = 1;
      read_memory();
    }
  }
  if (menu_level == 2) {
    *parametro = *encorder_value;
    int valor = *parametro;
    print_time2(valor, 4);
    Serial.print("parametro:");
    Serial.println((int)*parametro);
    if (checa_bot_confirm()) {
      writeEepromInt(*parametro, pos_memory);
      lcd.setCursor(0, 1);
      lcd.print("                ");
      menu_level = 1;
      *encorder_value = retorno;
      encoderMin = 0;
      factor = 1;
    }
    if (checa_bot_exit()) {
      lcd.setCursor(0, 1);
      lcd.print("                ");
      menu_level = 1;
      *encorder_value = retorno;
      encoderMin = 0;
      factor = 1;
    }
  }
}
//-----------------------------------------------------------------------------
//Funçao para configuraçao dos parametros do PID.
void confing_param_PID(char *texto_1, volatile int *encorder_value, int *parametro, int pos_memory, int retorno) {

  lcd.setCursor(0, 0);
  lcd.print(texto_1);
  if (menu_level == 1) {
    if (checa_bot_enter()) {
      menu_level = 2;
      *encorder_value = *parametro;
      encoderMax = 10000;
      factor = 10;
    }
    if (checa_bot_exit()) {
      lcd.setCursor(0, 0);
      lcd.print("                ");
      menu_level = 0;
      *encorder_value = 0;
      factor = 1;
      read_memory();
    }
  }
  if (menu_level == 2) {
    encoderMin = 0;
    *parametro = *encorder_value;
    int valor = *parametro;
    print_PID(valor, 4);
    if (checa_bot_confirm()) {
      writeEepromInt(*parametro, pos_memory);
      lcd.setCursor(0, 1);
      lcd.print("                ");
      menu_level = 1;
      *encorder_value = retorno;
      encoderMin = 0;
      factor = 1;
    }
    if (checa_bot_exit()) {
      lcd.setCursor(0, 1);
      lcd.print("                ");
      menu_level = 1;
      *encorder_value = retorno;
      encoderMin = 0;
      factor = 1;
    }
  }
}
//-----------------------------------------------------------------------------
//Funçao para leitura da temperatura do LM35
float read_LM35() {
  int reading = analogRead(LM35);
  float temperatureC = (reading * (3.3 / 4096.0)) * 100;
  return temperatureC;
}
//-----------------------------------------------------------------------------
//Funçao para tratar o sinal de Zerocrossing do sinal AC.
void zeroCrossing() {
  if (power_triac >= 5 && output_power_on)
    zeroCross_signal = true;
}
//-----------------------------------------------------------------------------
//Funçao para controle de disparo do TRIAC.
void Update_IT_callback(void) {  // Toggle pin. 10hz toogle --> 5Hz PWM
  //digitalWrite(PB10, !digitalRead(PB10));
  int power = 100 - power_triac;
  if (output_power_on) {
    if (power >= 10) {  // if (power >= 10 && output_power_on) {
      if (zeroCross_signal) {
        valor_triac++;
        if (valor_triac > power) {
          digitalWrite(triac_pin, HIGH);
          if (valor_triac > (power + pulso_triac)) {
            digitalWrite(triac_pin, LOW);
            valor_triac = 0;
            zeroCross_signal = false;
          }
        }
      }
    } else digitalWrite(triac_pin, HIGH);
  } else digitalWrite(triac_pin, LOW);
}
//-----------------------------------------------------------------------------
//Funçao para temporizaçao de 1 segundo atraver de interrupçao.
void time_seconds() {
  digitalWrite(PB10, !digitalRead(PB10));
  if (tempo_segundos_modo) {
    *tempo_segundos = *tempo_segundos + 1;
  } else {
    *tempo_segundos = *tempo_segundos - 1;
    if (*tempo_segundos == 0) {
      MyTim2->pause();
    }
  }
}
//-----------------------------------------------------------------------------
//Configuraçao dos TIMERs
void timer_init() {
  MyTim->setOverflow(80, MICROSEC_FORMAT);  // 10 Hz
  MyTim->attachInterrupt(Update_IT_callback);
  MyTim->resume();
  //******
  MyTim2->setOverflow(1000000, MICROSEC_FORMAT);  // 10 Hz
  MyTim2->attachInterrupt(time_seconds);
  MyTim2->pause();
  //MyTim2->resume();
  attachInterrupt(PA8, zeroCrossing, RISING);
}
//-----------------------------------------------------------------------------
//Funçao para tratar o sinal do encorder atraver de interrupçao.
void encoder() {
  //detachInterrupt(PB4);
  if (state == HIGH) {
    state = LOW;
  } else {  // state must be LOW
    state = HIGH;
  }
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime >= 5) {  //
    //delay(10);
    if (digitalRead(PB3) == LOW) {
      encoderPosition = encoderPosition - factor;
      *point_encoderPosition = *point_encoderPosition - factor;
    } else {
      encoderPosition = encoderPosition + factor;
      *point_encoderPosition = *point_encoderPosition + factor;
    }
    //--------------------------------------------
    if (*point_encoderPosition < encoderMin) {
      *point_encoderPosition = encoderMin;
    }
    if (*point_encoderPosition > encoderMax) {
      *point_encoderPosition = encoderMax;
    }
    if (encoderPosition < encoderMin) {
      encoderPosition = encoderMin;
    }
    if (encoderPosition > encoderMax) {
      encoderPosition = encoderMax;
    }
  }
  //Serial.print("encoderPosition:");
  //Serial.println(encoderPosition);
}
//-----------------------------------------------------------------------------
//Funçao para mostrar corrretamento o parametro TIME no Display
void print_time(int time, int posiçao) {

  int aux = time / 60;
  if (aux >= 10) {
    lcd.setCursor(posiçao, 1);
  } else {
    lcd.setCursor(posiçao, 1);
    lcd.print("0");
  }
  lcd.print(aux);
  lcd.print(":");

  aux = time - (aux * 60);
  if (aux >= 10) {
    lcd.setCursor((posiçao + 3), 1);
  } else {
    lcd.setCursor((posiçao + 3), 1);
    lcd.print("0");
  }
  lcd.print(aux);
  lcd.print("s");
}
//-----------------------------------------------------------------------------
//Funçao para mostrar corrretamento o parametro TIME no Display
void print_time2(int time, int posiçao) {

  int time_h;
  int time_m;
  int time_s;

  time_h = (time / 3600);
  time_m = (time - (3600 * time_h)) / 60;
  time_s = (time - (3600 * time_h) - (time_m * 60));
  //03:25:15
  lcd.setCursor(posiçao, 1);
  if (time_h >= 10) {
    lcd.print(time_h);
  } else {
    lcd.print("0");
    lcd.print(time_h);
  }
  lcd.print(":");
  //.................................
  if (time_m >= 10) {
    lcd.print(time_m);
  } else {
    lcd.print("0");
    lcd.print(time_m);
  }
  lcd.print(":");
  //.................................
  if (time_s >= 10) {
    lcd.print(time_s);
  } else {
    lcd.print("0");
    lcd.print(time_s);
  }
}
//-----------------------------------------------------------------------------
//Funçao para mostrar corrretamento o parametro TEMPERATURA no Display
void print_temp(float temp, int posiçao, int line) {
  if (temp >= 1000) {
    lcd.setCursor(posiçao, line);
  } else if (temp >= 100) {
    lcd.setCursor(posiçao, line);
    lcd.write(LCD_SPACE_SYMBOL);
  } else if (temp >= 10) {
    lcd.setCursor(posiçao, line);
    lcd.write(LCD_SPACE_SYMBOL);
    lcd.write(LCD_SPACE_SYMBOL);
  } else {
    lcd.setCursor(posiçao, line);
    lcd.write(LCD_SPACE_SYMBOL);
    lcd.write(LCD_SPACE_SYMBOL);
    lcd.write(LCD_SPACE_SYMBOL);
  }
  lcd.print(temp, 1);
  lcd.write(LCD_DEGREE_SYMBOL);
  lcd.print(F("C"));
  lcd.write(LCD_SPACE_SYMBOL);
}
//-----------------------------------------------------------------------------
//Funçao para mostrar corrretamento o parametro PID no Display
void print_PID(float pid_value, int posiçao) {
  //Serial.println(temp);
  // = 0.01 = 1/1000
  //0.01 = 1
  //10.00 = 10000
  if (pid_value >= 10000) {
    lcd.setCursor(posiçao, 1);
  } else {
    lcd.setCursor(posiçao, 1);
    lcd.write(LCD_SPACE_SYMBOL);
  }
  lcd.print(pid_value / 1000, 2);
}
//-----------------------------------------------------------------------------
//Rotina de acionamento do BUZZER
void bip() {
  digitalWrite(Buzzer, HIGH);  //ROTINA PARA ACIONAMENTO DO BUZZER
  delay(100);
  digitalWrite(Buzzer, LOW);
}
//-----------------------------------------------------------------------------
//Menu de configuraçao.
void menu_confing() {
  if (menu_level == 1) {
    M1A = encoderPosition;
    if (M1A > 12) {
      encoderPosition = 12;
      M1A = 12;
    }
  }

  switch (M1A) {
    case 0:
      {
        confing_param_temp(TxT_01, &encoderPosition, 0, &Preheat_Temp, 0x00, 0);
        break;
      }
    case 1:
      {
        confing_param_time(TxT_02, &encoderPosition, &Preheat_Time, 0x02, 1);
        break;
      }
    case 2:
      {
        confing_param_temp(TxT_03, &encoderPosition, Preheat_Temp, &Soaking_Temp, 0x04, 2);
        break;
      }
    case 3:
      {
        confing_param_time(TxT_04, &encoderPosition, &Soaking_Time, 0x06, 3);
        break;
      }

    case 4:
      {
        confing_param_temp(TxT_05, &encoderPosition, Soaking_Temp, &Reflow_Temp, 0x08, 4);
        break;
      }
    case 5:
      {
        confing_param_time(TxT_06, &encoderPosition, &Reflow_Time, 0x0A, 5);
        break;
      }
    case 6:
      {
        confing_param_temp(TxT_07, &encoderPosition, 0, &cooling_Temp, 0x0E, 6);
        break;
      }
    case 7:
      {
        confing_param_time(TxT_08, &encoderPosition, &cooling_Time, 0x0C, 7);
        break;
      }
    case 8:
      {
        confing_param_temp(TxT_09, &encoderPosition, 0, &stufa_Temp, 0x10, 8);
        break;
      }
    case 9:
      {
        confing_param_time(TxT_10, &encoderPosition, &stufa_Time, 0x12, 9);
        break;
      }
    case 10:
      {
        confing_param_PID(TxT_11, &encoderPosition, &pid_kp, 0x14, 10);
        break;
      }
    case 11:
      {
        confing_param_PID(TxT_12, &encoderPosition, &pid_ki, 0x16, 11);
        break;
      }
    case 12:
      {
        confing_param_PID(TxT_13, &encoderPosition, &pid_kd, 0x18, 12);
        break;
      }
  }
}
//-----------------------------------------------------------------------------
void reflow() {
  //Fonçao principal do controlador - controle do processo de reflow.
  //char *TxT_14 = (char *)"Preheat..110.0°C";
  // |Preheat..110.0°C|
  // |0:25:00..045.2°C|
  //int Preheat_Temp;
  //int Preheat_Time;
  //int Soaking_Temp;
  //int Soaking_Time;
  //int Reflow_Temp;
  //int Reflow_Time;
  //int cooling_Time;
  //int cooling_Temp;

  //Estagio - Preheat
  //Estagio - Soaking
  //Estagio - Reflow
  //Estagio - cooling

  static int aux_menu_reflow;
  static bool process_start = false;
  static int menucontrolreflow;
  int bot_enter = checa_bot_confirm2();

  if (menu_level == 1) {
    tempo_segundos = &total_time_process;  // carrega o enderesso da varivel de tempo;
    encoderPosition = 0;
    tempo_segundos_modo = true;            //modo de contagem crecente.
    lcd.setCursor(0, 0);
    lcd.print(TxT_14);
    total_time_process = Preheat_Temp + Preheat_Time + Soaking_Time + Reflow_Time;
    time_process = Preheat_Time;
    menu_level = 2;
    aux_menu_reflow = 0;
  }

  if (bot_enter == 2) {
    aux_menu_reflow = 0;
    total_time_process = 0;
    process_start = true;
    Setpoint = Preheat_Temp;
    MyTim2->resume();
  }

  if (!process_start) {
    menucontrolreflow++;
    if (menucontrolreflow > 6) {
      aux_menu_reflow++;
      menucontrolreflow = 0;
      if (aux_menu_reflow > 3) {
        aux_menu_reflow = 0;
      }
    }
  }

  //---------------
  //bloco de  controle
  if (process_start) {
    digitalWrite(PB0, !digitalRead(PB0));
    output_power_on = true;

    if (aux_menu_reflow == 0 && total_time_process > time_process) {  //Preheat
      aux_menu_reflow = 1;
      time_process = time_process + Soaking_Time;
      Setpoint = Soaking_Temp;
      bip();
      delay(200);
    }
    if (aux_menu_reflow == 1 && total_time_process > time_process) {  //Soaking
      aux_menu_reflow = 2;
      time_process = time_process + Reflow_Time;
      Setpoint = Reflow_Temp;
      bip();
      delay(200);
      bip();
      delay(200);
    }
    if (aux_menu_reflow == 2 && total_time_process > time_process) {  //reflow
      aux_menu_reflow = 3;
      time_process = time_process + cooling_Time;
      Setpoint = cooling_Temp;
      digitalWrite(cooling_pin, HIGH);
      output_power_on = false;
      bip();
      delay(200);
      bip();
      delay(200);
      bip();
      delay(200);
    }
    if (aux_menu_reflow == 3 && total_time_process > time_process) {  //cooling
      MyTim2->pause();
      digitalWrite(cooling_pin, LOW);
      lcd.clear();
      lcd.print(" END OF PROCESS ");
      process_start = false;
      digitalWrite(PB0, LOW);
      bip();
      bip();
      bip();
      bip();
      bip();
      bip();
      delay(2000);
      lcd.setCursor(0, 0);
      lcd.print("                 ");
    }
    //------------------
  }

  if (aux_menu_reflow == 0) {
    lcd.setCursor(0, 0);
    lcd.print(TxT_14);
    if (!process_start) {
      print_time2(Preheat_Time, 0);
    } else print_time2(total_time_process, 0);
    print_temp(Preheat_Temp, 8, 0);
    print_temp(temperature, 8, 1);
  }
  if (aux_menu_reflow == 1) {
    lcd.setCursor(0, 0);
    lcd.print(TxT_15);
    if (!process_start) {
      print_time2(Soaking_Time, 0);
    } else print_time2(total_time_process, 0);
    print_temp(Soaking_Temp, 8, 0);
    print_temp(temperature, 8, 1);
  }
  if (aux_menu_reflow == 2) {
    lcd.setCursor(0, 0);
    lcd.print(TxT_16);
    if (!process_start) {
      print_time2(Reflow_Time, 0);
    } else print_time2(total_time_process, 0);
    print_temp(Reflow_Temp, 8, 0);
    print_temp(temperature, 8, 1);
  }
  if (aux_menu_reflow == 3) {
    lcd.setCursor(0, 0);
    lcd.print(TxT_17);
    if (!process_start) {
      print_time2(cooling_Time, 0);
    } else print_time2(total_time_process, 0);
    print_temp(cooling_Temp, 8, 0);
    print_temp(temperature, 8, 1);
  }

  if (checa_bot_exit()) {
    if (process_start) {
      process_start = false;
      output_power_on = false;
      lcd.clear();
      lcd.print(" STOP PROCESS ");
      delay(1000);
      lcd.setCursor(0, 0);
      lcd.print("                 ");
      lcd.setCursor(0, 0);
      lcd.print(TxT_18);
    } else {
      point_encoderPosition = &aux;
      menu_level = 0;
      encoderPosition = 0;
      encoderMin = 0;
      factor = 1;
      process_start = false;
      output_power_on = false;
      lcd.clear();
      lcd.print(" END OF PROCESS ");
      delay(1000);
    }
  }
}
//-----------------------------------------------------------------------------
void stufa() {
  //estagio onde a temperatura sera constante em um determindo tempo.
  static int aux;
  int bot_enter = checa_bot_confirm2();
  static bool process_start = false;
  if (menu_level == 1) {
    encoderPosition = 0;
    lcd.setCursor(0, 0);
    lcd.print(TxT_18);
    encoderPosition = stufa_Time;
    factor = 30;
    menu_level = 2;
    encoderMax = 18000;
    aux = 0;
  }

  if (menu_level == 2 && aux == 0) {

    point_encoderPosition = &stufa_Time;

    if (bot_enter == 1) {
      aux = 1;
      encoderPosition = stufa_Temp;
      bot_enter = 0;
      encoderMax = 450;
      factor = 1;
    }
    if (bot_enter == 2) {
      process_start = true;
      MyTim2->resume();
      tempo_segundos = &stufa_Time;
      tempo_segundos_modo = false;
    }
  }

  if (menu_level == 2 && aux == 1) {  //temperatura
    point_encoderPosition = &stufa_Temp;
    if (bot_enter == 1) {
      aux = 0;
      encoderPosition = stufa_Time;
      bot_enter = 0;
      factor = 60;
      encoderMax = 18000;
      menu_level = 2;
    }
    if (bot_enter == 2) {
      process_start = true;
      MyTim2->resume();
      tempo_segundos = &stufa_Time;
      tempo_segundos_modo = false;
    }
  }

  //...................................................................
  if (process_start) {
    digitalWrite(PB0, !digitalRead(PB0));
    Setpoint = stufa_Temp;
    //________________________________________
    output_power_on = true;
    //________________________________________
    if (*tempo_segundos == 0) {
      output_power_on = false;
      bip();
      delay(200);
      bip();
      delay(200);
      bip();
      lcd.clear();
      lcd.print(" END OF PROCESS ");
      delay(1000);
      lcd.setCursor(0, 0);
      lcd.print(TxT_18);
      process_start = false;
      digitalWrite(PB0, LOW);
    }
  }

  print_time2(stufa_Time, 0);
  print_temp(stufa_Temp, 8, 0);
  print_temp(temperature, 8, 1);
  if (aux == 0) {
    lcd.setCursor(6, 1);
  } else {
    lcd.setCursor(11, 0);
  }
  //add stop timer
  if (checa_bot_exit()) {
    if (process_start) {
      process_start = false;
      output_power_on = false;
      MyTim2->pause();
      lcd.clear();
      lcd.print(" STOP PROCESS ");
      delay(1000);
      lcd.setCursor(0, 0);
      lcd.print(TxT_18);
    } else {
      point_encoderPosition = &aux;
      *tempo_segundos = stufa_Time;
      menu_level = 0;
      encoderPosition = 0;
      encoderMin = 0;
      factor = 1;
      process_start = false;
      output_power_on = false;
      lcd.clear();
      lcd.print(" END OF PROCESS ");
      delay(1000);
    }
  }
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  //delay(5000);
  //eeprom_format();
  point_encoderPosition = &aux;  //inicializaçao do ponteiro para evitar erros

  timer_init();

  analogReadResolution(12);
  pinMode(PB3, INPUT);
  pinMode(PB4, INPUT);
  pinMode(buton_exit, INPUT);
  pinMode(boton_enter, INPUT);
  pinMode(PC13, OUTPUT);
  pinMode(led_01, OUTPUT);
  pinMode(led_02, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(triac_pin, OUTPUT);
  pinMode(cooling_pin, OUTPUT);
  pinMode(zero_crossing_pin, INPUT);
  pinMode(PC13, OUTPUT);
  
  //------------------
  digitalWrite(led_01, HIGH);
  digitalWrite(led_02, HIGH);
  digitalWrite(Buzzer, HIGH);
  digitalWrite(cooling_pin, HIGH);
  delay(100);
  digitalWrite(led_01, LOW);
  digitalWrite(led_02, LOW);
  digitalWrite(Buzzer, LOW);
  digitalWrite(cooling_pin, LOW);
  //------------------
  delay(5000);
  read_memory();
  print_eeprom_memory();
  attachInterrupt(PB4, encoder, RISING);
  lcd.begin();
  lcd.backlight();
  lcd.print(" REFLOW - STM32 ");
  delay(2000);
  myMAX6675.begin();
 
  lcd.clear();
  lcd.cursor();
  //delay(5000);
  Serial.print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
  myPID.SetOutputLimits(0, 100);  // max output 100%
  myPID.SetMode(AUTOMATIC);
  Setpoint = 0;
  upd_pid_parametros();
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

int aux_loop;
void loop() {

  if (millis() - lastPublishMillis > 200) {
    lastPublishMillis = millis();
    temperature = myMAX6675.getTemperature();
    aux_temp = read_LM35();

    if (output_power_on) {
      Input = temperature;
      myPID.Compute();
      power_triac = Output;

    }

    if (power_triac > 100) {
      power_triac = 0;
    }

    if (menu_level == 0) {
      M0A = encoderPosition;
      if (M0A > 2) {
        encoderPosition = 0;
        M0A = 0;
      }
    }

    if (aux_temp > 40.0 || temperature > 400.0) { //Rotina de seguraça para evitar sobrearquecimento do sistema.
      output_power_on = false;
      menu_level = 0;
      M0A = 3;
    }

    switch (M0A) {
      case 0:
        {
          if (menu_level == 0) {
            lcd.setCursor(0, 0);
            lcd.print(" REFLOW         ");
            if (checa_bot_enter()) {
              menu_level = 1;
              lcd.clear();
            }
          } else
            reflow();
          break;
        }
      case 1:
        {
          if (menu_level == 0) {
            lcd.setCursor(0, 0);
            lcd.print(" ESTUFA         ");
            if (checa_bot_enter()) {
              menu_level = 1;
              lcd.clear();
            }
          } else
            stufa();
          break;
        }

      case 2:
        {
          if (menu_level == 0) {
            lcd.setCursor(0, 0);
            lcd.print(" CONFIGURACAO   ");
            if (checa_bot_enter()) {
              menu_level = 1;
              lcd.clear();
              encoderPosition = 0;
            }
          } else
            menu_confing();
          break;
        }
      case 3:
        {
          if (menu_level == 0) {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(" ERRO - TEMP   ");
            bip();
            bip();
            bip();
            bip();
            bip();
            bip();
            delay(1000);
            bip();
            bip();
            bip();
            bip();
            bip();
            while (1) {
            digitalWrite(PB0, HIGH);
            }
          }
          break;
        }
        aux_loop = 0;
    }
  }
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
