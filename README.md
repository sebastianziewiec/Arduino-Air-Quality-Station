// -----------------------------------------------------------------------------------------
// Program dla miernika temperatury i jakosci powietrza z wykorzystaniem technologii Arduino
// wykonany przez: Sebastian Ziewiec
// WSZ Edukacja Informatyka ciag MI 
// nr. albumu 24875 
// -----------------------------------------------------------------------------------------

// 1. Dolaczanie niezbednych bibliotek

#include "dht.h"                // biblioteka czujnika DHT22
dht DHT22;
#include <Wire.h>               // standardowa biblioteka Arduino 
#include <LiquidCrystal_I2C.h>  // biblioteka I2C dla LCD

LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); //Ustawienie adresu I2C na 0x3F

// 2. Definiowanie pinow i zmiennych 

#define DHT22PIN     2        // pin DHT22
#define PIN_LED      7        // pin PM2.5 ILED
#define PIN_ANALOG   0        // pin PM2.5 AOUT
#define RED_1        3        // pin diody czerwonej 1
#define RED_2        5        // pin diody czerwonej 2
#define YELLOW_1     6        // pin diody zoltej
#define GREEN_1      9        // pin diody zielonej 1
#define GREEN_2      10       // pin diody zielonej 2

#define MIN_VOLTAGE   600     // mv - prog dolnego zakresu napiecia dla braku pylu czujnika PM2.5
#define VREF          5000    // mv - napiecie referencyjne komparatora
#define MAX_ITERS     25      // liczba pomiarow do sredniej wartosci stezenia pylu PM2.5

// 3. Definiowanie zmiennych dla czujnika pylu PM2.5

int ADC_VALUE;
int ITER;
float VOLTAGE;
float DUST;
float AVG_DUST;

// 4. Funkcja setup

void setup () 

{

  Serial.begin(9600);               // inicjalizacja monitora szeregowego
  lcd.begin(16,2);                  // inicjalizacja LCD
  lcd.backlight();                  // zalaczenie podswietlenia LCD
  lcd.setCursor(0,0);               // ustawienie kursora w 1 wierszu 1 kolumnie

  pinMode(PIN_LED, OUTPUT);         // ustawienie pinu diody ILED jako wyjscie
  digitalWrite(PIN_LED, LOW);       // ustawienie stanu niskiego na diodzie ILED
  pinMode(RED_1, OUTPUT);           // ustawienie pinu diody swiecacej czerwonej 1 jako wyjscie
  digitalWrite (RED_1, LOW);        // ustawienie stanu niskiego na diodzie czerwonej 1
  pinMode(RED_2, OUTPUT);           // ustawienie pinu diody swiecacej czerwonej 2 jako wyjscie
  digitalWrite(RED_2, LOW);         // ustawienie stanu niskiego na diodzie czerwonej 2
  pinMode(YELLOW_1, OUTPUT);        // ustawienie pinu diody swiecacej zoltej jako wyjscie
  digitalWrite(YELLOW_1, LOW);      // ustawienie stanu niskiego na diodzie zoltej
  pinMode(GREEN_1, OUTPUT);         // ustawienie pinu diody swiecacej zielonej 1 jako wyjscie 
  digitalWrite(GREEN_1, LOW);       // ustawienie stanu niskiego diody swiecacej zielonej 1 
  pinMode(GREEN_2, OUTPUT);         // ustawienie pinu diody swiecacej zielonej 2 jako wyjscie
  digitalWrite(GREEN_2, LOW);       // ustawienie stanu niskiego diody zielonej 2 

}

float computeDust()                 // funkcja odczytujaca napiecie ADC i migajaca dioda podczerwona

{
  digitalWrite(PIN_LED, HIGH);
  delayMicroseconds(280);
  ADC_VALUE = analogRead(PIN_ANALOG);
  digitalWrite(PIN_LED, LOW);

  VOLTAGE = ADC_VALUE * (VREF / 1024.0) * 2,5;   // przeliczenie napiecia na mv. 

  if (VOLTAGE > MIN_VOLTAGE)                  // obliczanie PM2.5 jezeli zmierzone napiecie wykracza ponad wartosc progowa
  {
    return (VOLTAGE - MIN_VOLTAGE) * 0.2;
  }

  return 0;
  }
  
// 5. Funkcja loop

  void loop() 

  {
    int chk = DHT22.read(DHT22PIN);         // deklaracja nazwy zmiennej dla odczytów czujnika DHT22
    AVG_DUST = 0;                           //odczytana wartosc z ADC
    ITER = 0;
    
 while (ITER < MAX_ITERS)
    {
      DUST = computeDust();

      if (DUST > 0)
      {
        AVG_DUST += DUST;                 // liczenie do sredniej pomiaru PM2.5 tylko prawidlowych pomiarow
        ITER++;
        delay(50);
      }
    }

    AVG_DUST /= MAX_ITERS;

    Serial.print("D = ");               
    Serial.print(AVG_DUST);                    // wyswietlenie wyniku pomiaru na monitorze szeregowym
    Serial.print("ug/m3");

    delay(2500); 

  switch (chk)                                 // instrukcje dla czujnika DHT22
  {
    case DHTLIB_OK: 
    Serial.print("OKt"); 
    break;
    case DHTLIB_ERROR_CHECKSUM: 
    Serial.println("Błąd sumy kontrolnej"); 
    break;
    case DHTLIB_ERROR_TIMEOUT: 
    Serial.println("Koniec czasu oczekiwania - brak odpowiedzi"); 
    break;
    default: 
    Serial.println("Nieznany błąd"); 
    break;
  }
  Serial.print("Wilgotnosc (%): ");                  
  Serial.print((float)DHT22.humidity, 2);            // wyswietlenie wartosci wilgotnosci powietrza
  Serial.print("--");
  Serial.print("Temperatura (C): ");          
  Serial.println((float)DHT22.temperature, 2);      //wyswietlenie wartosci temperatury powietrza
 
  delay(500);  
  
// 6. Wyswietlenie wynikow pomiarow na wyswietlaczu LCD   
                             
lcd.setCursor(0,0);               //ustawienie kursora wyswietlacza w pierwszej linii
lcd.print("H%:"); 
lcd.print(DHT22.humidity);        // wyswietlenie wilgotnosci
lcd.print("TC:");
lcd.print(DHT22.temperature);     //wyswietlenie temperatury
lcd.setCursor(0,1);               // ustawienie kursora wyswietlacza w drugiej linii
lcd.print("PM2.5ug/m3:");
lcd.print(AVG_DUST);              // wyswietlenie zmierzonej zawartosci pylu PM2.5

// 7. Informacja o stanie jakosci powietrza pokazana za pomoca diod swiecacych 

if (AVG_DUST <=13 ){              // instrukcja wlaczajaca diody zielone 1 i 2 dla bardzo dobrego stanu jakosci powietrza 
  digitalWrite(RED_1, LOW);
  digitalWrite(RED_2, LOW);
  digitalWrite(YELLOW_1, LOW);
  digitalWrite(GREEN_1, LOW);
  digitalWrite(GREEN_2, HIGH);
}
  else if (AVG_DUST > 13 && AVG_DUST <=35) {        //instrukcja wlaczajaca diode zielona 1 dla dobrego stanu jakosci powietrza
    digitalWrite(RED_1, LOW);
  digitalWrite(RED_2, LOW);
  digitalWrite(YELLOW_1, LOW);
  digitalWrite(GREEN_1, HIGH);
  digitalWrite(GREEN_2, LOW);
  }
  else if (AVG_DUST > 35 && AVG_DUST <=75) {        // instrukcja wlaczajaca diode zolta dla sredniego stanu jakosci powietrza
    digitalWrite(RED_1, LOW);
  digitalWrite(RED_2, LOW);
  digitalWrite(YELLOW_1, HIGH);
  digitalWrite(GREEN_1, LOW);
  digitalWrite(GREEN_2, LOW);
  }
  else if (AVG_DUST > 75 &&  AVG_DUST <= 110) {      // instrukcja wlaczajaca diode czerwona 2 dla zlego stanu jakosci powietrza
    digitalWrite(RED_1, LOW);
  digitalWrite(RED_2, HIGH);
  digitalWrite(YELLOW_1, LOW);
  digitalWrite(GREEN_1, LOW);
  digitalWrite(GREEN_2, LOW);
  }
  else if (AVG_DUST > 110) {                        // instrukcja wlaczajaca diody czerwone 1 i 2 dla bardzo zlego stanu jakosci powietrza 
    digitalWrite(RED_1, HIGH);
  digitalWrite(RED_2, LOW);
  digitalWrite(YELLOW_1, LOW);
  digitalWrite(GREEN_1, LOW);
  digitalWrite(GREEN_2, LOW);
  }
}
  


   
