#include <SPI.h>
#include "alti_logger.h"

const String altiLogger_ID = "DLG_LOGGER_20090605";
const String altiLogger_ver = "v0.2";
const String altiLogger_file_extension = ".csv";

boolean zatrzymaj = false;
// Variables:
int previousMillis = 0;        // will store the last time the LED was updated
int interval = 1000;           // interval at which to blink (in milliseconds)

uint32_t FlashAddres;

uint8_t strona[256];
uint8_t costam = 0;


// ---------------------------------------------------
//
//      ...::: SETUP :::...
//
// ---------------------------------------------------
void setup() 
{
  
  uint8_t i;
  
  // initialize alitLogger
  i = altiLogger_init();
  if(i != 0) altiLogger_error(i);
  
  //if (sprawdzMode()==1) modePC(); //connection to PC
  delay(100);
  Serial.begin(SERIAL_BAUD);
  delay(200);
  
  // find first free data Flash addres
  FlashAddres = Flsh_start();
  if(FlashAddres == 0 )         //"Flash mem is not formatted"
  {
    if(altiLogger_error(3)==true)
    {
      modePC();
    }
    else altiLogger_error(100); //wrong altiLogger_error return value
  }
  
  ///*
  Serial.print("*****");
  Serial.print(FlashAddres);
  Serial.println("*****");
  //*/
  
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
    zmienna = measurement_to_FlashStringFormat(); //stwórz ciąg z danych
    Serial.print(zmienna); //wyświetl ciąg
    //FlashAddres = writeFlashString(FlashAddres,zmienna); //zapisz ciąg do Flash
    
    if (Serial.available() > 0) { //sprawdz czy nie ma komendy
      input = Serial.readString();
      if (input.indexOf("PC") > -1)modePC();
    }
    
    timeCount = timeCount + interval; //przelicz daną "czas"
    LEDping(1);
  }
  
}//end of loop


// ---------------------------------------------------
//
//      Flsh_print_mem - print all write mem
//
// ---------------------------------------------------
void Flsh_print_mem()
{
  
  // print first 64 mem byte
  Serial.println("\r\n------------------------------");
  while(FlashBusy());
  selectFlash();
    
  SPI.transfer(Flash_CMD_read_data);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  uint8_t j = 0;
  for(uint32_t i = 0; i < Flash_FAT_0_addres; i++)
  {
    Serial.write(SPI.transfer(0x00));
    if(j==15){
      Serial.println("");
      j = 0;
    }
    else j++;
  }
  Serial.println("------------------------------");
  j = 0;
  for(uint32_t i = 0; i < Flash_FAT_0_addres; i++)
  {
    Serial.write(SPI.transfer(0x00));
    if(j==15){
      Serial.println("");
      j = 0;
    }
    else j++;
  }
  Serial.println("------------------------------");
  deselectFlash();

  // print all write memory
  
  
  
}//end of Flsh_print_mem


// ---------------------------------------------------
//
//      Flsh_Find_Start_Adress & write new file name
//
// ---------------------------------------------------
uint32_t Flsh_start()
{
  
  uint32_t addr;
  String ciag1 = "";
  String ciag2 = "";

  uint32_t return_value;
  String nazwa_pliku = "(1" + altiLogger_file_extension + ")";

  // porównanie altiLogger_ID z zapisem na początku pamięci
  for(addr = 0; addr < altiLogger_ID.length(); addr++)
  {
    ciag1 = ciag1 + Flash_read_8bit(addr);
    ciag2 = ciag2 + uint8_t(altiLogger_ID[addr]);
  }
  if(ciag1 != ciag2) return 0;
  
  Serial.print(altiLogger_ID + " ");
  Serial.println(altiLogger_ID.length());
  
  /*if(Flash_read_8bit(0x00)==0xFF){
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
    nazwa_pliku = "(" + String(i) + altiLogger_file_extension + ")";*/

  //znajdz start adress

  return 64;
  
  
}//end of Flsh_Find_Start_Adress


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
  Serial.print(altiLogger_ver);
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
      if (input.indexOf("print") > -1)Flsh_print_mem();
    }
  }
  
}//end of modePC


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
