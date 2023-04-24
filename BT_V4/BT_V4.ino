#include <ArduinoBlue.h>
#include <SharpIR.h>
#include <QMC5883LCompass.h>
#include <Servo.h>
#include <SoftwareSerial.h>

const unsigned long BAUD_RATE = 9600;
const int BLUETOOTH_TX = 3;     // Bluetooth TX -> Arduino 3
const int BLUETOOTH_RX = 2;     // Bluetooth RX -> Arduino 2
SoftwareSerial bluetooth(BLUETOOTH_TX, BLUETOOTH_RX);

Servo serwo;
QMC5883LCompass compass;
  int text ;
  char cmd[10];
  int cmdIndex;
  int predkosc=0;         //stara predkosc dla silnika
  int predkosc_new = 0;   //nowa predkosc, odczytana z BT
  int znak;

int kierunek_prawo = 8;
int kierunek_lewo = 7;
int predkosc_prawo = 11;
int predkosc_lewo = 10;

  int alarm1=0;
  int alarm2=0;
  int pozycja = 90;

int czujnik1 = A0;
int czujnik2 = A1;
int czujnik3 = A2;
int odleglosc1[20];
int odleglosc2[20];
int odleglosc3[20];
float cm1, cm2, cm3;
int RSSI=0; 
int kol = 0;
float suma, kolizja_lewo, kolizja_prawo = 0.0;   

int i=0;
String input;
byte a,b,c,d,e;

SharpIR Sharp_lewo(czujnik2, 1080);
SharpIR Sharp_prawo(czujnik3, 1080);

  void execute(char* cmd) {

       if(cmd[0] == 'w') {znak = 5; }  //pobranie prędkości
  else if(cmd[0] == 'r') znak = 4;  //reset
  else if(cmd[0] == 'a') znak = 10; // tryb automatycznego podążania
  else if(cmd[0] == '1') znak = 6;  //start silnika
  else if(cmd[0] == '0') znak = 7;  //stop
  else if(cmd[0] == '5') znak = 8;  //skręt w lewo
  else if(cmd[0] == '6') znak = 9;  //skręt w prawo
  else if(cmd[0] == '9') znak = 11;
  else if(cmd[0] == 'B') RSSI = atoi(cmd+1);  //tryb jazdy wstecz   
  else return; // unknown command

  predkosc_new = atoi(cmd+1);   //odczytanie wartości tablicy od znaku nr 1
    sterowanie();
  }

void sterowanie(){
    if (znak == 4){
       Serial.println("Reset ");        //start i reset pracy
       predkosc_new = 0;
      // serwo.write(90);
       praca_silnika();
      }
    if (znak == 5){                     //zmiana prędkości
       if(predkosc_new == 0){
        znak =7;
        }else{
       praca_silnika();
       //Serial.print("Predkosc= "); Serial.println(predkosc);
        }
      }
    if (znak == 6){
      praca_silnika();                  //jazda i śledzenie
      odczyt_kompasu();
       // Serial.println("Jedz: ");
      }
    if (znak == 7){                     //zatrzymanie pojazdu
       Serial.println("STOP");
       predkosc_new = 0;
        digitalWrite(kierunek_prawo, LOW);
        digitalWrite(kierunek_lewo, LOW);
        analogWrite(predkosc_prawo,predkosc);
        analogWrite(predkosc_lewo, 0);
    }
    if (znak == 8){
      pozycja -= 15;
      serwo.write(pozycja);
      praca_silnika();                  //skręt w lewo
      }
    if (znak == 9){
      pozycja += 15;
      serwo.write(pozycja);
      praca_silnika();                  //skręt w prawo
      }
    if (znak == 10){
      tryb_auto();                      //włączenie tryby auto
      }
    if (znak == 11){
      tryb_wsteczny();                  //włączenie tryby wstecz
      }
  
  
  }

void test_srednia() { 
                                                                //średnia krocząca czujników odległości
  odleglosc1[19] = analogRead(czujnik1);
  odleglosc2[19] = Sharp_lewo.distance(); 
  odleglosc3[19] = Sharp_prawo.distance(); 
  for (int i = 0; i <19; i++) {
    odleglosc1[i] = odleglosc1[i+1];
    odleglosc2[i] = odleglosc2[i+1];
    odleglosc3[i] = odleglosc3[i+1];
    suma+=odleglosc1[i];
    kolizja_lewo += odleglosc2[i];
    kolizja_prawo += odleglosc3[i];
  }
  cm1 = suma / 40;
  cm2 = kolizja_lewo / 20.0;
  cm3 = kolizja_prawo / 20.0;
  suma = 0.0;
  kolizja_lewo = 0.0;
  kolizja_prawo = 0.0;
  //Serial.print("czujnik środek = ");
 // Serial.print(cm1);
  //Serial.print("   czujnik lewo = ");
 // Serial.print(cm2);
  //Serial.print("   czujnik prawo = ");
 // Serial.println(cm3);
  delay(20);
}
void kolizja(){
 if(cm1 <=50 || cm2 <=50 || cm3 <=50 )       //mniej niż 50cm z każdej strony to stop
      {predkosc=0;
      kol = 1;
     }else kol = 0;
  
    if(cm1<=60){
      if(predkosc>=2){              //mniej niz 60 cm to zwolnij
        predkosc-=2;      
        } 
      }

}
void tryb_auto(){

  while( znak == 10){
 
      if (bluetooth.available()!=0) {                           //Oczekiwanie na przyjście danych z Bluetooth
           
          if (bluetooth.available()>0) {
             a = bluetooth.read();
              cmd[0] = a;
            delay(1);
            b = bluetooth.read();
             cmd[1] = b;
            delay(1);
            c = bluetooth.read();
             cmd[2] = c;
            delay(1);
            d = bluetooth.read();
             cmd[3] = d;
            delay(1);
            e = bluetooth.read();
             cmd[4] = e;
           }    
         execute(cmd);   
           
      }
    

   
        /*
        for(int i=0; i<20; i++){
        odleglosc1[i] = analogRead(czujnik1);  
        odleglosc2[i] = Sharp_lewo.distance(); 
        odleglosc3[i] = Sharp_prawo.distance();  
        suma += odleglosc1[i];
        kolizja_lewo += odleglosc2[i];
        kolizja_prawo += odleglosc3[i];
        }
        
        cm1 = suma/40.0;                             //wartosc srednia czujnika na wprost
        cm2 = kolizja_lewo / 20.0;
        cm3 = kolizja_prawo / 20.0;
        suma = 0.0;
        kolizja_lewo = 0.0;
        kolizja_prawo = 0.0;
        //Serial.println("cm1=");
        // Serial.println(cm1);
        //Serial.print("cm2= ");
        //Serial.print(cm2);
        //Serial.print("cm3= ");
        //Serial.print(cm3);
        */
        //if(cm3 <= 60) predkosc = 0;
      
   
   
   test_srednia();
    kolizja();
    
      if(RSSI>30 && RSSI < 60 && kol == 0){  
    kolizja();
     if(cm1<=70){           //70cm = przyspieszanie do 20             
      predkosc=25;
        }
    else if(cm1<=85){       //85cm = przyspieszanie do 30     
      predkosc=35;
      }
    else if(cm1<=100){      //100cm = przyspieszanie do 40           
      predkosc=45;
        }
    else if(cm1<=130){      //130cm = przyspieszanie do 50      
      predkosc=50;
        }
    else if(cm1>450){  
      if(predkosc>=5){      //powyżej 4,5m (obiekt nie wykryty) samochód zwalnia
        predkosc-=1;
        } 
        
   }}

    if(RSSI>=60 && RSSI < 65){            //odleglosc z RSSI wieksza niż 1,5 m
   
    predkosc = 50;
    }
    else if(RSSI>=65 && RSSI < 70){       //odleglosc z RSSI wieksza niż 2,5 m
   
    predkosc = 60;
    }
    else if(RSSI>=70 && RSSI < 75){       //odleglosc z RSSI wieksza niż 4,5 m
 
    predkosc = 70;
   } 
   else if(RSSI>=75 && RSSI <= 80){      //odleglosc z RSSI wieksza niż 6,5m

    predkosc = 75;
   }else if(RSSI>80){                    //odleglosc z RSSI wieksza niż 8 m
    if(predkosc>=5){              
      predkosc-=1;
      }
    }
     kolizja();
      digitalWrite(kierunek_prawo, HIGH);
      digitalWrite(kierunek_lewo, HIGH);
      analogWrite(predkosc_prawo,predkosc);
      analogWrite(predkosc_lewo, 0);
     
  }
      
}

void tryb_wsteczny(){
     // serwo.write(90);                                                  //blokada przednich kół
      digitalWrite(kierunek_prawo, HIGH);
      digitalWrite(kierunek_lewo, HIGH);
      analogWrite(predkosc_prawo,0);
      analogWrite(predkosc_lewo, 40);                                     //prędkosc jazdy w tył 
 
  }
  
void odczyt_kompasu(){
  
  int x, y, z;
  
  // Read compass values
  compass.read();

  // Return XYZ readings
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();
  float heading = atan2(y, x);
  
   float declinationAngle = 0.099;
  heading += declinationAngle;
  if(heading < 0)
    heading += 2*PI;
    
  if(heading > 2*PI)
    heading -= 2*PI;
   
  float headingDegrees = heading * 180/M_PI; 
  
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  
  }

void praca_silnika(){
  
  if(((predkosc_new + 50) < predkosc) && (predkosc_new != 0)){

    while(predkosc > predkosc_new+5){      
      predkosc -=5;
      digitalWrite(kierunek_prawo, HIGH);
      digitalWrite(kierunek_lewo, HIGH);
      analogWrite(predkosc_prawo,predkosc);
      analogWrite(predkosc_lewo, 0);
       }
     }else if(((predkosc_new + 50) < predkosc) && (predkosc_new == 0)){
       predkosc_new = 0;
        digitalWrite(kierunek_prawo, LOW);
        digitalWrite(kierunek_lewo, LOW);
        analogWrite(predkosc_prawo,predkosc);
        analogWrite(predkosc_lewo, 0);
      }
     else{
     
  predkosc = abs(predkosc_new);                                                 //przypisanie wartosci z BT

    digitalWrite(kierunek_prawo, HIGH);
    digitalWrite(kierunek_lewo, HIGH);
    analogWrite(predkosc_prawo,predkosc);
    analogWrite(predkosc_lewo, 0);

     }
  }


void setup() {
  Serial.begin(BAUD_RATE);
  bluetooth.begin(BAUD_RATE);
  compass.init();
  pinMode(kierunek_prawo, OUTPUT);
  pinMode(kierunek_lewo, OUTPUT);
  pinMode(predkosc_prawo, OUTPUT);
  pinMode(predkosc_lewo, OUTPUT);
  pinMode(5, INPUT); //alarm lewo
  pinMode(6, INPUT); //alarm prawo
  pinMode(czujnik1, INPUT);
  pinMode(czujnik2, INPUT);
  pinMode(czujnik3, INPUT);
  serwo.attach(9);
  pozycja = 90;
  serwo.write(pozycja);
  
  for(int i=0; i<5; i++){
  odleglosc1[i] = analogRead(czujnik1);  
  }
  cm1 =( odleglosc1[0]+odleglosc1[1]+odleglosc1[2]+odleglosc1[3]+odleglosc1[4] )/5;   //wartosc srednia czujnika na wprost
  
}

void loop() {
/*
      if (bluetooth.available()!=0) {                           //Oczekiwanie na przyjście danych z Bluetooth
           String input = bluetooth.readStringUntil("\r");
          for (int i = 0; i < input.length(); i++) {
          cmd[i] = input.charAt(i);
           }    
         execute(cmd);   
         Serial.println(cmd);    
      }
*/
if (bluetooth.available()!=0) {                           //Oczekiwanie na przyjście danych z Bluetooth
           
          if (bluetooth.available()>0) {
             a = bluetooth.read();
              cmd[0] = a;
            delay(1);
            b = bluetooth.read();
             cmd[1] = b;
            delay(1);
            c = bluetooth.read();
             cmd[2] = c;
            delay(1);
            d = bluetooth.read();
             cmd[3] = d;
            delay(1);
            e = bluetooth.read();
             cmd[4] = e;
           }    
         execute(cmd);   
         Serial.println(cmd);    
         //a=0; b=0; c=0; d=0; e=0;
     }

/*                                                            PROGRAM DLA MODUŁU BLUETOOTH HC-06 I APLIKACJI ROBOREMO
    if(Serial.available()>0) {              
    char c = (char)Serial.read();
    if(c=='\r' || c=='\n') {
      cmd[cmdIndex] = 0;
      cmdIndex = 0;
     execute(cmd);
    } else {
      cmd[cmdIndex++] = c;
    }
    }
*/
    
    
}
