#include <AccelStepper.h> //Dodanie bibliotek zwi¹zanych ze sterowaniem silnikami krokowymi
#include <AFMotor.h>


AF_Stepper motor1(200, 1);
AF_Stepper motor2(200, 2);  //Konfiguracja silników krokowych


const int stepper1speed = 100 * 2;  
const int stepper1acc = 50 * 6;
const int stepper2speed = 200 * 2;
const int stepper2acc = 100 * 6;




void forwardstep1() {
  motor1.onestep(FORWARD, SINGLE);  //Definiowanie pojedynczego kroku
}
void backwardstep1() {
  motor1.onestep(BACKWARD, SINGLE);
}

void forwardstep2() {
  motor2.onestep(FORWARD, SINGLE);
}
void backwardstep2() {
  motor2.onestep(BACKWARD, SINGLE);
}

AccelStepper stepper1(forwardstep1, backwardstep1); //Dodanie silników do klasy AccelStepper
AccelStepper stepper2(forwardstep2, backwardstep2);


int prawo ; //Zmienne pomocnicze wy³¹czników krañcowych
int lewo ;
int gora ;
int dol ;

bool flaga_gora = 1;
bool flaga_dol = 1;
bool flaga_prawo = 1;
bool flaga_lewo = 1;

bool flaga_stopu = 1;
bool flaga_powrotu = 0;
bool flaga_ostatniej_pozycji = 1;
bool flaga_timera = 1;
bool flaga_strzelania = 0;

bool START = 0;

int obrot = 1;   //Zmienne pomocnicze obrotu kolumny
int x = 0;

int wychyl = 1; //Zmienne pomocnicze wychy³u ko³yski
int y = 0;


char inData[20]; // Zapewnienie lokacji w pamiêci danym wejœciowym
char inChar; // Zmienna do przechowywania znaku
byte index = 0; // Index do oznaczania, gdzie przechowaæ znak
int liczba = 0; // Zmienna pomocnicza do liczenia odebranych liczb

long val1 = 0;  //Zmienne pomocnicze do zapisu odebranych wartoœci
long val2 = 0;
long lastKnownX = 0;
long lastKnownY = 0;
int pozycja = 0;

int licznik = 2000;
int licznik2 = 0;
long czas = 0;
long czas_start = 0;
long czas_koniec = 0;
long stary_czas = 0;

int ilosc_trafien = 0;

void setup() //Funkcja ykonywana jednorazowo, na pocz¹tku dzia³ania programu
{
  stepper1.setMaxSpeed(stepper1speed);        //Ustawienie maksymalnej prêdkoœci...
  stepper1.setAcceleration(stepper1acc);  //... oraz przyspieszenia silników

  stepper2.setMaxSpeed(stepper2speed);
  stepper2.setAcceleration(stepper2acc);

  Serial.begin(9600);  //Otwarcie portu komunikacji

  pinMode(A0, OUTPUT);
  pinMode(A5, OUTPUT);

  digitalWrite(A0, LOW);
  digitalWrite(A5, LOW);
}

void loop() //Funkcja wykonywana w pêtli
{
  prawo = analogRead(1);
  lewo = analogRead(2);
  gora = analogRead(3);
  dol = analogRead(4);

  while (Serial.available() > 0) // Oczekiwanie na zwolnienie siê portu komunikacji
  {
    if (index < 19) // Jeden mniej ni¿ wymiar tablicy inData
    {
      inChar = Serial.read(); // Zczytaj znak
      inData[index] = inChar; // Przechowaj
      index++; // PrzejdŸ na kolejne miejsce
      inData[index] = '\0'; // Zakoñcz
    }
    START = 1;
  }
  index = 0; //Wyzerowanie indeksu

  char *str = inData, *p = str;   // Stworzenie dodatkowych zmiennych
  while (*p) {                      // Tak d³ugo, jak jest wiêcej danych do przetworzenia
    if (isdigit(*p) || *p == '-') { // Przy znalezieniu cyfry lub znaku "-"
      if (liczba < 3) {           // Jeœli nie wczytano jeszcze trzech liczb
        if (liczba == 0) { // Jeœli to pierwsza liczba
          val1 = strtol(p, &p, 10); // Zczytaj znak i go zapisz
          liczba++;
        }
        if (liczba == 1) { // Jeœli to druga liczba
          val2 = strtol(p, &p, 10); // Zczytaj znak i go zapisz
          // Serial.print("  Uchyb pionowy: ");
          // Serial.println(val2);   // Wydrukuj zmienn¹
          liczba++;
        }
      }
    } else // W przeciwnym wypadku, przejdŸ do kolejnego miejsca
      p++;
    if (liczba == 2) // Jeœli zczytano ju¿ dwie liczby
      liczba = 0;  // Przygotuj siê do zczytania kolejnych dwóch
  }


  if (START) {
    if (val1 == 998 || val2 == 998) {
      stepper1.setMaxSpeed(0);
      stepper1.setAcceleration(0);
      stepper2.setMaxSpeed(0);
      stepper2.setAcceleration(0);
      x = 0;
      y = 0;
      START = 0;
      ilosc_trafien = 0;
      exit;
    } else if (ilosc_trafien > 3) {

      digitalWrite(A0, LOW);
      digitalWrite(A5, LOW);

      if (dol != 0 ) {
        stepper2.move(50);
        stepper2.run();
      } else {
        stepper1.setMaxSpeed(0);
        stepper2.setMaxSpeed(0);
        stepper1.move(0);
        stepper2.move(0);
        flaga_powrotu = 1;
      }
    } else if (flaga_powrotu) {
      if (dol == 0 ) {
        stepper1.setMaxSpeed(stepper1speed);
        stepper1.setAcceleration(stepper1acc);
        stepper2.setMaxSpeed(stepper2speed);
        stepper2.setAcceleration(stepper2acc);
        stepper2.move(-50);
        stepper2.run();
      } else {
        flaga_powrotu = 0;
      }
    } else {


      stepper1.setMaxSpeed(stepper1speed);
      stepper1.setAcceleration(stepper1acc);
      stepper2.setMaxSpeed(stepper2speed);
      stepper2.setAcceleration(stepper2acc);

      if (val1 != 999 && val2 != 999) {
        licznik = 0;
        flaga_ostatniej_pozycji = 1;

        if (flaga_stopu) {
          stepper1.setMaxSpeed(0);
          stepper1.setAcceleration(0);
          x = 0;
          stepper1.setMaxSpeed(stepper1speed);
          stepper1.setAcceleration(stepper1acc);

          stepper2.setMaxSpeed(0);
          stepper2.setAcceleration(0);
          y = 0;
          stepper2.setMaxSpeed(stepper2speed);
          stepper2.setAcceleration(stepper2acc);

          flaga_stopu = 0;
        }

        lastKnownX = (val1) * 0.3;
        lastKnownY = (-val2) * 0.7;
        stepper1.move(lastKnownX);
        stepper2.move(lastKnownY);

      } else {
        if (licznik >= 2000) {

          flaga_stopu = 1;

          if (licznik < 3000)
            pozycja = 1000;
          else {
            if (licznik < 4000)
              pozycja = 0;
            else
              licznik = 2000;
          }

          if (obrot == 1)
            x = pozycja;
          if (obrot == -1)
            x = -pozycja;
          if (wychyl == 1)
            y = pozycja;
          if (wychyl == -1)
            y = -pozycja;

          stepper1.move(x);
          stepper2.move(y);

          licznik++;
        } else {
          licznik ++;

          stepper1.setMaxSpeed(stepper1speed);
          stepper1.setAcceleration(stepper1acc);
          stepper2.setMaxSpeed(stepper2speed);
          stepper2.setAcceleration(stepper2acc);

          if (flaga_ostatniej_pozycji) {
            stepper1.move(lastKnownX);
            stepper2.move(lastKnownY);
            flaga_ostatniej_pozycji = 0;
          }
        }
      }



      /////    \/ STRZELANIE   //////////////////////////////////////////////////////////////////////////////////////////////////////////////

      if ((abs(val1) < 7 && abs(val2) < 7) || flaga_strzelania == 1) {
        if (flaga_timera) {
          czas_start = millis();
          flaga_timera = 0;
        }
        czas_koniec = millis();
      } else {
        flaga_timera = 1;
        czas_start = 0;
        czas_koniec = 0;
      }
      stary_czas = czas;
      czas = czas_koniec - czas_start;

      if (czas >= 1000) {
        switch (czas - 1000) {
          case 0:
            flaga_strzelania = 1;
            digitalWrite(A0, LOW);
            digitalWrite(A5, HIGH);
            break;
          case 100:
            digitalWrite(A5, LOW);
            break;
          case 200:
            digitalWrite(A0, HIGH); //strza³
            break;
          case 350:
            digitalWrite(A0, LOW);
            digitalWrite(A5, HIGH);
            break;
          case 500:
            digitalWrite(A5, LOW);
            licznik = 2000;
           // licznik2 = 0;
            if(flaga_strzelania)
              ilosc_trafien++;
            flaga_strzelania = 0;
            break;
          default: break;
        }
        //licznik2++;
      } else {
        //licznik2 = 0;
        digitalWrite(A0, LOW);
        digitalWrite(A5, LOW);
      }

      /////    /\ STRZELANIE   //////////////////////////////////////////////////////////////////////////////////////////////////////////////



      /////    \/ KRAÑCÓWKI   //////////////////////////////////////////////////////////////////////////////////////////////////////////////


      if (prawo > 50) {
        if (flaga_prawo) {
          stepper1.setMaxSpeed(0);
          stepper1.setAcceleration(0);
          x = 0;
          stepper1.setMaxSpeed(stepper1speed);
          stepper1.setAcceleration(stepper1acc);
          x = 1000;
          flaga_prawo = 0;
        }
        obrot = 1;
        val1 = 999;
        val2 = 999;
      } else
        flaga_prawo = 1;


      if (lewo > 50 ) {
        if (flaga_lewo) {
          stepper1.setMaxSpeed(0);
          stepper1.setAcceleration(0);
          x = 0;
          stepper1.setMaxSpeed(stepper1speed);
          stepper1.setAcceleration(stepper1acc);
          x = -1000;
          flaga_lewo = 0;
        }
        obrot = -1;
        val1 = 999;
        val2 = 999;
      } else
        flaga_lewo = 1;


      if (gora < 50 ) {
        if (flaga_gora) {
          stepper2.setMaxSpeed(0);
          stepper2.setAcceleration(0);
          y = 0;
          stepper2.setMaxSpeed(stepper2speed);
          stepper2.setAcceleration(stepper2acc);
          y = 1000;
          flaga_gora = 0;
        }
        wychyl = 1;
        val1 = 999;
        val2 = 999;
      } else
        flaga_gora = 1;

      if (dol < 50 ) {
        if (flaga_dol) {
          stepper2.setMaxSpeed(0);
          stepper2.setAcceleration(0);
          y = 0;
          stepper2.setMaxSpeed(stepper2speed);
          stepper2.setAcceleration(stepper2acc);
          y = -1000;
          flaga_dol = 0;
        }
        wychyl = -1;
        val1 = 999;
        val2 = 999;
      } else
        flaga_dol = 1;

      /////    /\ KRAÑCÓWKI   //////////////////////////////////////////////////////////////////////////////////////////////////////////////

      stepper1.run();
      stepper2.run();

    }
  }
}
