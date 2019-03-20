#include <SPI.h>
#include "alti_logger.h"

String verLogger = "v0.2";

boolean zatrzymaj = false;
// Variables:
int previousMillis = 0;        // will store the last time the LED was updated
int interval = 1000;            // interval at which to blink (in milliseconds)

uint32_t licznik = 32;  //pierwsze 32 bajty to licznik plików
double timeCount = 0;   //licznik milisecund od power on
String input;

uint8_t strona[256];
uint8_t costam = 0;


// ---------------------------------------------------
//
//      ...::: SETUP :::...
//
// ---------------------------------------------------

void setup() 
{

  initLogger();
  //if (sprawdzMode()==1) modePC(); //connection to PC
  delay(500);
  Serial.begin(SERIAL_BAUD);
  delay(500);

  
  
}//end of setup


// ---------------------------------------------------
//
//      ...::: LOOP :::...
//
// ---------------------------------------------------

void loop()
{
  String zmienna = "";
  
  if (millis() - previousMillis > interval){
    previousMillis = millis();
    previousMillis = previousMillis / interval;
    previousMillis = previousMillis * interval;

    readParameters(); //read MS5611 parameters
    zmienna = daneDoFlash(); //stwórz ciąg z danych
    Serial.println(zmienna); //wyświetl ciąg
    writeFlashDane(licznik,zmienna); //zapisz ciąg do Flash
    licznik = licznik + zmienna.length(); //przelicz adres
    timeCount = timeCount + interval; //przelicz daną "czas"
    if (Serial.available() > 0) { //sprawdz czy nie ma komendy
      input = Serial.readString();
      if (input.indexOf("PC") > -1)modePC();
    }
  }
}//end of loop


// ---------------------------------------------------
//
//      zwraca zformatowwane dane do zapisu do Flash
//
// ---------------------------------------------------
String daneDoFlash()
{
  double zmienna = 0;
  String dane = "";

  zmienna = timeCount;
  zmienna = zmienna/1000;
  dane = String(zmienna,3) + ";";
  zmienna = Temperature;
  zmienna = zmienna / 10;
  dane = dane + String(zmienna,1) + ";";
  zmienna = Altitude;
  zmienna = zmienna / 10;
  dane = dane + String(zmienna,1) + ";";

  return dane;
}
// ---------------------------------------------------
//
//      check PC connection
//
// ---------------------------------------------------

uint8_t sprawdzMode()
{
  uint16_t x = 0xB4A3;
  
  pinMode(serial_TX1, OUTPUT);
  pinMode(serial_RX1, INPUT_PULLUP);
  digitalWrite(pinLED, LOW);
  
  for (int8_t i=0;i<16;i++)
  {
    digitalWrite(serial_TX1, bitRead(x,i));
    if(digitalRead(serial_RX1)!=bitRead(x,i)) return 1;
  }
  return 0;
}//end of sprawdzMode()


// ---------------------------------------------------
//
//      PC mode
//
// ---------------------------------------------------
void modePC()
{

  
  
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Serial.print("...::: DLG LOGGER SPI MS5611 ");
  Serial.print(verLogger);
  Serial.println(" :::...");
  Serial.println("...:::          PC MODE           :::...");

  for(int i=0; i<0x100;i++){
    strona[i]=i*10;
  }

  Serial.print("Flash manufacturer ID: ");
  Serial.println(Flash_Manufacturer_ID(),HEX);

  /*readFlashPage(0x0000);
  wyswietl_strone();
  writeFlashTest(0x11);
  readFlashPage(0x0000);
  wyswietl_strone();*/

  zatrzymaj=true;
  while(1){
    if (zatrzymaj == false) {
      if (millis() - previousMillis > interval){
        previousMillis = millis();
        previousMillis = previousMillis / interval;
        previousMillis = previousMillis * interval;

        readParameters();
        
      
        LEDping(1);
      }
    }
    if (Serial.available() > 0) {
      input = Serial.readString();
      if (input.indexOf("stop") > -1)zatrzymaj = true;
      if (input.indexOf("start") > -1)zatrzymaj = false;
      if (input == "c")wyswietlCx();
      if (input.indexOf("read") > -1)wyswietl_strone();
      if (input.indexOf("erase") > -1)FlashErase();
      if (input.indexOf("identyfikator") > -1)Serial.print("DLG_LOGGER_20090605");
      if (input.indexOf("help") > -1)showLoggerHelp();
    }
  }
}//end of modePC

void showLoggerHelp(){
  Serial.print("DLG LOGGER ");
        Serial.println("v0.2");
        Serial.println("lista komend:");        
        Serial.println("   start - wyświetlania pomiarów");
        Serial.println("   stop - wyświetlania pomiarów");
        Serial.println("   read - ...");
        Serial.println("   erase - kasuj pamięć flash");
        Serial.println("   identyfikator - wyświatl");
        Serial.println("   help - to menu");
}

void wyswietl_strone()
{
  Serial.println("-----------------------------------------------");
  readFlashPage(0x0000);
  int j=0;
  for(int i=0;i<0x100;i++){
    wyswietl_liczbe(strona[i]);
    j++;
    if(j==16){
      Serial.println("");
      j=0;
    }
  }


  Serial.println("-----------------------------------------------");
}

void wyswietl_liczbe(uint8_t liczba)
{

  if(liczba<0x10){
    Serial.print("0");
    Serial.print(liczba,HEX);
  }
  else Serial.print(liczba,HEX);
  Serial.print(" ");
}

void readFlashPage(uint16_t pageNum)
{
  while(FlashBusy());
  uint8_t byteH = pageNum>>8;
  uint8_t byteL = pageNum;

  uint8_t dump;

  selectFlash();
  dump = SPI.transfer(0x03); // command read page
  dump = SPI.transfer(byteH);
  dump = SPI.transfer(byteL);
  dump = SPI.transfer(0);
  uint8_t i=0;
  do {
    strona[i] = SPI.transfer(0x00);
    i++;
  }while(i!=0);
  
 /* for(int i=0;i<0x100;i++){
    strona[i]= SPI.transfer(0);
  }*/
  
  deselectFlash();
}

void writeFlashDane(uint32_t addr, String dane){
  while(FlashBusy());
  uint8_t dump;
  Flash_Write_Enable();
  selectFlash();
  SPI.transfer(0x02);
  SPI.transfer(addr>>16);
  SPI.transfer(addr>>8);
  SPI.transfer(addr);
  for(int i = 0; i < dane.length(); i++){
    SPI.transfer(dane.charAt(i));
  }
  deselectFlash();
  Flash_Write_Disable();
}
