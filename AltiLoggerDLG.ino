#include <SPI.h>
#include "alti_logger.h"

#define altiLogger_ID "DLG_LOGGER_20090605"
String verLogger = "v0.2";
#define altiLogger_file_extension ".csv"

#define Flash_CMD_page_program  0x02
#define Flash_CMD_read_data     0x03
#define Flash_max_addres        4194303   // 32Mbit = 4Mbyte = 4 x 1024 x 1024 = 4194304 max adress 419304 - 1
#define Flash_FAT_0_addres      32
#define Flash_data_0_addres     64

boolean zatrzymaj = false;
// Variables:
int previousMillis = 0;        // will store the last time the LED was updated
int interval = 1000;            // interval at which to blink (in milliseconds)

uint32_t FlashAddres;  //pierwsze 32 bajty to licznik plików
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
  FlashAddres = Flsh_Find_Start_Adress();
  Serial.print("*****");
  Serial.print(FlashAddres);
  Serial.println("*****");
  
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
    Serial.print(zmienna); //wyświetl ciąg
    FlashAddres = writeFlashString(FlashAddres,zmienna); //zapisz ciąg do Flash
    
    if (Serial.available() > 0) { //sprawdz czy nie ma komendy
      input = Serial.readString();
      if (input.indexOf("PC") > -1)modePC();
    }
    
    timeCount = timeCount + interval; //przelicz daną "czas"
    
  }
  
}//end of loop


// ---------------------------------------------------
//
//      Flsh_Find_Start_Adress & write new file name
//
// ---------------------------------------------------
uint32_t Flsh_Find_Start_Adress()
{
  
  uint8_t i = 0;
  uint32_t addr;

  uint32_t return_value;
  String nazwa_pliku = "(1" + altiLogger_file_extension + ")";
  
  if(Flash_read_8bit(0x00)==0xFF){
    FlashErase();
    Flash_write_8bit(0x00,0x01);
    return_value = writeFlashString(Flash_data_0_addres, nazwa_pliku);
    return return_value;
  }
  else {

//nazwa nowego pliku
    addr = i / 8;
    while(bitRead(Flash_read_8bit(addr),(i - addr * 8)) != 1){
      i++;
      addr = i / 8;
    }
    nazwa_pliku = "(" + String(i) + altiLogger_file_extension + ")";

//znajdz start adress

    addr = Flash_data_0_addres;
    while(Flash_read_8bit(addr) != 0xFF){
      addr++;
    }
    return_value = writeFlashString(addr, nazwa_pliku);
    return return_value;
  }

  return 0;
  
}//end of Flsh_Find_Start_Adress

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
  dane = dane + String(zmienna,1) + "\r\n";

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
      if (input.indexOf("identyfikator") > -1)Serial.print(altiLogger_ID);
      if (input.indexOf("help") > -1)showLoggerHelp();
    }
  }
  
}//end of modePC


// ---------------------------------------------------
//
//      showLoggerHelp
//
// ---------------------------------------------------
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
  
}//end of showLoggerHelp


// ---------------------------------------------------
//
//      wyswietl_strone
//
// ---------------------------------------------------
void wyswietl_strone()
{

  //uint8_t dump;
  uint32_t addr=0;
  uint8_t i = 0;
  
  Serial.println("***********************************************");

  for(addr=0;addr<32;addr++)Serial.println(Flash_read_8bit(addr),BIN);

  Serial.println("-----------------------------------------------");

  while(Flash_read_8bit(addr)!=0xFF){
    Serial.write(Flash_read_8bit(addr));
    addr++;
  }
  
  /*readFlashPage(0x0000);
  
  for(int i=0;i<0x100;i++){
    if(strona[i]>0x7E)Serial.print(" ");
      else Serial.write(strona[i]);
  }
  Serial.println("**********");
  for(int i=0;i<0x100;i++){
    Serial.print(Flash_read_8bit(i),HEX);
    Serial.print(" ");
  }
  Serial.println("-----------------------------------------------");*/
  
}//end of wyswietl_strone


// ---------------------------------------------------
//
//      wyswietl_liczbe
//
// ---------------------------------------------------
void wyswietl_liczbe(uint8_t liczba)
{

  if(liczba<0x10){
    Serial.print("0");
    Serial.print(liczba,HEX);
  }
  else Serial.print(liczba,HEX);
  Serial.print(" ");
  
}//end of wyswietl_liczbe


// ---------------------------------------------------
//
//      Flash_read_8bit
//
// ---------------------------------------------------
uint8_t Flash_read_8bit(uint32_t addr)
{
  
  while(FlashBusy());

  uint8_t return_value;
  uint8_t byteH = addr >> 16;
  uint8_t byteM = addr >> 8;
  uint8_t byteL = addr;

  selectFlash();
  SPI.transfer(Flash_CMD_read_data);
  SPI.transfer(byteH);
  SPI.transfer(byteM);
  SPI.transfer(byteL);
  return_value = SPI.transfer(0x00);
  deselectFlash();
  
  return return_value;
  
}//end of Flash_read_8bit


// ---------------------------------------------------
//
//      Flash_write_8bit
//
// ---------------------------------------------------
void Flash_write_8bit(uint32_t addr, uint8_t data)
{
  
  while(FlashBusy());

  uint8_t byteH = addr >> 16;
  uint8_t byteM = addr >> 8;
  uint8_t byteL = addr;

  Flash_Write_Enable();
  selectFlash();
  SPI.transfer(Flash_CMD_page_program);
  SPI.transfer(byteH);
  SPI.transfer(byteM);
  SPI.transfer(byteL);
  SPI.transfer(data);
  deselectFlash();
  Flash_Write_Disable();
  
}//end of Flash_write_8bit


// ---------------------------------------------------
//
//      readFlashPage
//
// ---------------------------------------------------
void readFlashPage(uint16_t pageNum)
{
  
  while(FlashBusy());
  uint8_t byteH = pageNum>>8;
  uint8_t byteL = pageNum;

  uint8_t dump;

  selectFlash();
  dump = SPI.transfer(Flash_CMD_read_data); // command read page
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
  
}//end of readFlashPage


// ---------------------------------------------------
//
//      writeFlashString
//      zapisuje pod wskazany adres flash ciąg
//      i zwraca nowy dres flash
//
// ---------------------------------------------------
uint32_t writeFlashString(uint32_t addr, String str){
  
  while(FlashBusy());
  Flash_Write_Enable();
  selectFlash();
  SPI.transfer(Flash_CMD_page_program);       //Flash page program
  SPI.transfer(addr>>16);                 //High byte
  SPI.transfer(addr>>8);                  //Middle byte
  SPI.transfer(addr);                     //Low byte
  for(int i = 0; i < str.length(); i++){  //write str to Flash
    SPI.transfer(str.charAt(i));
  }
  deselectFlash();
  Flash_Write_Disable();
  addr = addr + str.length(); //przelicz adres
  return addr;
  
}//end of writeFlashString
