#include <SPI.h>

#define SERIAL_BAUD      115200

//definicja nr pinow
#define pinLED PC13
#define MS5611_SS PA0
#define FLASH_SS  PA1

#define MS5611_CMD_ADC_READ   0x00
#define MS5611_CMD_RESET      0x1E
#define MS5611_CMD_CONV_D1    0x48
#define MS5611_CMD_CONV_D2    0x58
#define MS5611_PROM_C1        0xA2
#define MS5611_PROM_C2        0xA4
#define MS5611_PROM_C3        0xA6
#define MS5611_PROM_C4        0xA8
#define MS5611_PROM_C5        0xAA
#define MS5611_PROM_C6        0xAC
#define MS5611_CMD_READ_PROM  0xA0

#define Flash_CMD_page_program  0x02
#define Flash_CMD_read_data     0x03
#define Flash_max_addres        4194303   // 32Mbit = 4Mbyte = 4 x 1024 x 1024 = 4194304 max adress 419304 - 1
#define Flash_FAT_0_addres      32
#define Flash_data_0_addres     64
#define Flash_manufacturer_ID   0xEF
#define Flash_W25Q32_ID_8bit    0x15

// ...::: SERIAL :::...
#define serial_TX1        PA9
#define serial_RX1        PA10

double referencePressure; // Ciśnienie początkowe
double timeCount = 0;
String dane_do_flash;
double a, b;

int MS5611_CONV_T = 10; //czas pomiaru MS5611
uint16_t C1,C2,C3,C4,C5,C6; //stałe kalibracyjne MS5611
uint32_t rawTemp, rawPress; 
int16_t Temp; //temperatura
uint32_t Press; //ciśnienie
int16_t Alti; //wysokość
boolean zatrzymaj = false;

// Variables:
int previousMillis = 0;        // will store the last time the LED was updated
int interval = 1000;            // interval at which to blink (in milliseconds)

String input;

String altiLogger_ID = "DLG_LOGGER_190525";
String altiLogger_ver = "v0.3";
String altiLogger_file_extension = ".csv";

uint32_t FlashAddres = 64; //Flsh start adress

// ---------------------------------------------------
//
//      ...::: SETUP :::...
//
// ---------------------------------------------------
void setup() 
{

  Serial.begin(SERIAL_BAUD);

  pinMode(pinLED, OUTPUT);
  LED_on();
  pinMode(MS5611_SS, OUTPUT);
  deselectMS5611();
  pinMode(FLASH_SS, OUTPUT);
  deselectFlash();
  delay(500);

  //Ustawienia SPI
  SPI.setBitOrder(MSBFIRST); // Set the SPI_1 bit order
  SPI.setDataMode(SPI_MODE0); //Set the  SPI_2 data mode 0
  SPI.setClockDivider(SPI_CLOCK_DIV8);      // Slow speed (72 / 16 = 4.5 MHz SPI_1 speed)
  SPI.begin();

  print_LoggerDLG_info();

  if(init_MS5611()==false){
    Logger_Error(2);
  }
  
  if(init_Flash()==false){
    Logger_Error(3);
  }
  
  if(Flash_mem_format()==false){
    Logger_Error(4);
  }

  //znalezienie początku wolnej pamięci
  FlashAddres = Flash_find_start_address();
  Serial.print("first flash free addres: ");
  Serial.println(FlashAddres);

  //sprawdzenie czy PCmode
  input = Serial.readString();
  Serial.println("autor: Hejnas");
  input = Serial.readString();
  if (input.indexOf("autor: Hejnas") > -1)
  {
    Serial.println("--- S T A R T ----");
  }
  else
  {
     PC_mode();
  }
  
  //odczytanie nr pliku    
  uint8_t nr_pliku = find_file_num();
  if(nr_pliku==0){
    Logger_Error(5);
  }
  
  //wpisanie nazwy nowego pliku do mem
  String nazwa_pliku;
  nazwa_pliku = String(nr_pliku) + altiLogger_file_extension  + "\r\n";
  FlashAddres = Flash_write_string(FlashAddres,nazwa_pliku);
  Serial.print("file name: " + nazwa_pliku);
  
}// end of setup


// ---------------------------------------------------
//
//      ...::: LOOP :::...
//
// ---------------------------------------------------
void loop()
{
  
  if (zatrzymaj == false)
  {
    if (millis() - previousMillis > interval)
    {
      previousMillis = millis();
      previousMillis = previousMillis / interval;
      previousMillis = previousMillis * interval;
      
      readParameters();

      timeCount = previousMillis;
      timeCount = timeCount / 1000;
      a = Alti;
      a = a / 10;
      b = Temp;
      b = b / 10;
      dane_do_flash = String(timeCount) + ";" + a + ";" + b + "\r\n";
      
      Serial.print(dane_do_flash);

      FlashAddres = Flash_write_string(FlashAddres,dane_do_flash); //zapisz ciąg do Flash
    
      LED_ping(1);

    }
  }
  
  //sheck serial commend "PC" avilable
  if (Serial.available() > 0) //sprawdz czy nie ma komendy
  {
    input = Serial.readString();
    if (input.indexOf("PC") > -1)
    {
      PC_mode();
    }
  }
  
}// end of loop


// ---------------------------------------------------
//
//      LoggerError
//
// ---------------------------------------------------
void Logger_Error(uint8_t error_num){

  String error_desc;
  uint8_t LEDping_count;

  switch (error_num){
    case 2:   //MS5611 is not connected
      error_desc = "error 2 - MS5611 not coected - check wired";
      LEDping_count = 2;
      break;
    case 3:   //Flash is not connected
      error_desc = "error 3 - Flash not coected - check wired";
      LEDping_count = 3;
      break;
    case 4:   //Flash is not formated
      error_desc = "error 4 - Flash not formated";
      LEDping_count = 4;
      break;
    case 5:   //Flash is not formated
      error_desc = "error 5 - błąd zapisu do FAT";
      LEDping_count = 5;
      break;
    default:
      error_desc = "error ??? - error return value altiLogger_error";
      LEDping_count = 24;
      break;
  }
  
  while(1){    
    Serial.print(error_desc);
    Serial.println(" - fixed it or more info send 'PC' command");
    LED_ping(LEDping_count); 
    delay(1000);

    //sheck serial commend "PC" avilable
    if (Serial.available() > 0) { //sprawdz czy nie ma komendy
      input = Serial.readString();
      if (input.indexOf("PC") > -1){
        PC_mode();
      }
    }
  }
  
}//end off LoggerError


// ---------------------------------------------------
//
//      wyświetla info o wersi loggera
//
// ---------------------------------------------------
void PC_mode()
{
  LED_on();
  Serial.println(" --- PC mode ---");
  print_LoggerDLG_info();
  
  while(1)
  {
    if (Serial.available() > 0) {
      input = Serial.readString();
      if (input.indexOf("erase") > -1)FlashErase();
      if (input.indexOf("addID") > -1)Flash_write_string(0x00,altiLogger_ID);
      if (input.indexOf("print") > -1)Flash_print_mem();
      if (input.indexOf("temp") > -1){
        readConv();
        Serial.print("Teperature=");
        Serial.println(calcTemperature(),DEC);
      }
      if (input.indexOf("press") > -1){
        readConv();
        Serial.print("Pressure=");
        Serial.println(calcPressure(),DEC);
      }
      if (input.indexOf("alti") > -1){
        readConv();
        Serial.print("Altitude=");
        Serial.println(calcAltitude(Press, referencePressure),DEC);
      }
    }
  }
}//end off PC_mode


// ---------------------------------------------------
//
//      wyświetla info o wersi loggera
//
// ---------------------------------------------------
void print_LoggerDLG_info()
{
  
  Serial.println("....::::  " + altiLogger_ID + "  " + altiLogger_ver + "  ::::....");
  
}// end of print_DLG_info()


// ---------------------------------------------------
//
//      obsługa LEDa
//
// ---------------------------------------------------

void LED_ping(uint8_t LEDping_count)
{
  if(LEDping_count==1){
    LED_on();
    delay(50);
    LED_off();
  }
  else{
    for(uint8_t i=0;i<LEDping_count;i++){
      LED_on();
      delay(50);
      LED_off();
      delay(200);
    }
  }
}// end of LED_ping


void LED_on()
{
  digitalWrite(pinLED, LOW);
}//end of LED_on


void LED_off()
{
  digitalWrite(pinLED, HIGH);
}//end of LED_off

/* ---------------------------------------------------
 *
 *      select and deselect MS5611 and Flash
 *
 * ---------------------------------------------------*/
void selectMS5611(){ digitalWrite(MS5611_SS, LOW);}
void deselectMS5611(){ digitalWrite(MS5611_SS, HIGH);}
void selectFlash(){ digitalWrite(FLASH_SS, LOW);}
void deselectFlash(){ digitalWrite(FLASH_SS, HIGH);}


// ---------------------------------------------------
//
//      
//
// ---------------------------------------------------

bool init_MS5611(void){

  for(uint8_t i = 0; i < 5; i++)
  {
    LED_off();
    MS5611_write(MS5611_CMD_RESET); //reset MS5611
    delay(500);
    LED_on();
    delay(500);    
  }
  
  //odczytanie danych kalibracyjnych
  C1 = MS5611_read_16bits(MS5611_PROM_C1);
  C2 = MS5611_read_16bits(MS5611_PROM_C2);
  C3 = MS5611_read_16bits(MS5611_PROM_C3);
  C4 = MS5611_read_16bits(MS5611_PROM_C4);
  C5 = MS5611_read_16bits(MS5611_PROM_C5);
  C6 = MS5611_read_16bits(MS5611_PROM_C6);
  
  if((C1==0x0000) || (C1==0xFFFF)) return false;
  if((C2==0x0000) || (C2==0xFFFF)) return false;
  if((C3==0x0000) || (C3==0xFFFF)) return false;
  if((C4==0x0000) || (C4==0xFFFF)) return false;
  if((C5==0x0000) || (C5==0xFFFF)) return false;
  if((C6==0x0000) || (C6==0xFFFF)) return false;
  
  //odczytanie referencePressure
  readConv();
  referencePressure = calcPressure();
  
  //wyswietlCx();

  return true;
  
}// end of init_MS5611


// ---------------------------------------------------
//
//      
//
// ---------------------------------------------------

bool init_Flash(void){

  selectFlash();
  SPI.transfer(0x90); //read ID command
  SPI.transfer(0);
  SPI.transfer(0);
  SPI.transfer(0);
  if(SPI.transfer(0)!=Flash_manufacturer_ID){
    deselectFlash();
    return false;
  }
  if(SPI.transfer(0)!=Flash_W25Q32_ID_8bit){
    deselectFlash();
    return false;
  }
  deselectFlash();  
  return true;
  
}// end of init_Flash


void wyswietlCx(){
  Serial.print("C1: ");
  Serial.print(C1);
  Serial.print("; C2: ");
  Serial.print(C2);
  Serial.print("; C3: ");
  Serial.print(C3);
  Serial.print("; C4: ");
  Serial.print(C4);
  Serial.print("; C5: ");
  Serial.print(C5);
  Serial.print("; C6: ");
  Serial.println(C6);
}


// ---------------------------------------------------
//
//      Check mem format
//
// ---------------------------------------------------
byte Flash_mem_format(){
  
//Flash check logger_id at the beginning of memory
//if != ltiLogger_ID then return 4 //error 4 "Flash memory not formated"
  //Serial.println("check mem format");
  uint8_t buffer_ID;
  uint8_t read_byte;
  uint8_t FlashCheck = true;
  selectFlash();
  SPI.transfer(Flash_CMD_read_data);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  for(uint8_t i=0;i<altiLogger_ID.length();i++){ 
    read_byte = SPI.transfer(i);
    buffer_ID = altiLogger_ID[i];
//    Serial.print(buffer_ID,HEX);
//    Serial.print(" ");
//    Serial.println(read_byte,HEX);
    if(buffer_ID!=read_byte){
      deselectFlash();
      return false;
    }
  }
  deselectFlash();
  return true;
  
}//end of Check_mem_format


// ---------------------------------------------------
//
//      odczytaj rawTemp i rawPress z MS5611
//
// ---------------------------------------------------

void readParameters(){

  // odczyt D1 i D2
  readConv();

  // obliczanie parametrów
  Temp = calcTemperature();
  Press = calcPressure();
  Alti = calcAltitude(rawPress,referencePressure);
}

void readConv(void){

  //odczyt raw ciśnienia
  MS5611_write(MS5611_CMD_CONV_D1);
  delay(MS5611_CONV_T);
  rawPress = MS5611_read_24bits(MS5611_CMD_ADC_READ);

  //odczyt raw temperatury
  MS5611_write(MS5611_CMD_CONV_D2);
  delay(MS5611_CONV_T);
  rawTemp = MS5611_read_24bits(MS5611_CMD_ADC_READ);
  
}// end of readSensors


// ---------------------------------------------------
//
//      odczytaj 8 bity z MS5611
//
// ---------------------------------------------------

uint8_t MS5611_read_8bits(uint8_t reg)
{
  uint8_t dump;
  uint8_t return_value;
  uint8_t addr = reg; // | 0x80; // Set most significant bit
  digitalWrite(MS5611_SS, LOW);
  dump = SPI.transfer(addr);
  return_value = SPI.transfer(0);
  digitalWrite(MS5611_SS, HIGH);
  return return_value;
}


// ---------------------------------------------------
//
//      odczytaj 16 bity z MS5611
//
// ---------------------------------------------------

uint16_t MS5611_read_16bits(uint8_t reg)
{
  uint8_t dump, byteH, byteL;
  uint16_t return_value;
  uint8_t addr = reg; // | 0x80; // Set most significant bit
  digitalWrite(MS5611_SS, LOW);
  dump = SPI.transfer(addr);
  byteH = SPI.transfer(0);
  byteL = SPI.transfer(0);
  digitalWrite(MS5611_SS, HIGH);
  return_value = ((uint16_t)byteH<<8) | (byteL);
  return return_value;
}


// ---------------------------------------------------
//
//      odczytaj 24 bity z MS5611
//
// ---------------------------------------------------

uint32_t MS5611_read_24bits(uint8_t reg)
{
  uint8_t dump,byteH,byteM,byteL;
  uint32_t return_value;
  uint8_t addr = reg;
  digitalWrite(MS5611_SS, LOW);
  dump = SPI.transfer(addr);
  byteH = SPI.transfer(0);
  byteM = SPI.transfer(0);
  byteL = SPI.transfer(0);
  digitalWrite(MS5611_SS, HIGH);
  return_value = (((uint32_t)byteH)<<16) | (((uint32_t)byteM)<<8) | (byteL);
  return return_value;
}


// ---------------------------------------------------
//
//      zapisz 8 bitów do MS5611
//
// ---------------------------------------------------

void MS5611_write(uint8_t reg)
{
  uint8_t dump;
  digitalWrite(MS5611_SS, LOW);
  dump = SPI.transfer(reg);
  digitalWrite(MS5611_SS, HIGH);
}


// ---------------------------------------------------
//
//      kalkulacja ciśnienia MS5611
//
// ---------------------------------------------------
uint32_t calcPressure()
{
    uint32_t D1 = rawPress;

    uint32_t D2 = rawTemp;
    int32_t dT = D2 - (uint32_t)C5 * 256;

    int64_t OFF = (int64_t)C2 * 65536 + (int64_t)C4 * dT / 128;
    int64_t SENS = (int64_t)C1 * 32768 + (int64_t)C3 * dT / 256;

  int32_t TEMP = 2000 + ((int64_t) dT * C6) / 8388608;

  int64_t OFF2 = 0;
  int64_t SENS2 = 0;

  if (TEMP < 2000)
  {
      OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
      SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
  }

  if (TEMP < -1500)
  {
      OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
      SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
  }

  OFF = OFF - OFF2;
  SENS = SENS - SENS2;

    double P = (D1 * SENS / 2097152 - OFF) / 32768;

    uint32_t PRS = uint32_t(P);

    return P;
}

// ---------------------------------------------------
//
//      kalkulacja temperatury MS5611
//
// ---------------------------------------------------
int16_t calcTemperature()
{
    uint32_t D2 = rawTemp;
    int32_t dT = D2 - (uint32_t)C5 * 256;

    int32_t TEMP = 2000 + ((int64_t) dT * C6) / 8388608;

    int64_t TEMP2 = 0;

  if (TEMP < 2000)
  {
      TEMP2 = (dT * dT) / (2 << 30);
  }

    TEMP = TEMP - TEMP2;

    return ((int16_t)TEMP/10);
}

// ---------------------------------------------------
//
//      kalkulacja wysokości MS5611
//
// ---------------------------------------------------
int16_t calcAltitude(double pressure, double seaLevelPressure)
{
    //double seaLevelPressure = 101325;
    double alti = (44330.0f * (1.0f - pow((double)Press / (double)seaLevelPressure, 0.1902949f)));
    alti=alti*10;
    return alti;
}


/*  ***********************************************************************************
 *  
 *  Obsluga Flash
 *  
 *  ***********************************************************************************
 */


 /* ---------------------------------------------------
 *
 *      flash write enable
 *
 * ---------------------------------------------------*/
void Flash_Write_Enable()
{
  uint8_t dump;
  selectFlash();
  dump = SPI.transfer(0x06);
  deselectFlash();
}//end of Flash_Write_Enable


/* ---------------------------------------------------
 *
 *      flash write disable
 *
 * ---------------------------------------------------*/
void Flash_Write_Disable()
{
  uint8_t dump;
  selectFlash();
  dump = SPI.transfer(0x04);
  deselectFlash();
}//end of Flash_Write_Disable


/* ---------------------------------------------------
 *
 *      check busy Flash
 *
 * ---------------------------------------------------*/
bool FlashBusy()
{
 
  uint8_t dump;
  selectFlash();
  SPI.transfer(0x05);
  dump = SPI.transfer(0xff);
  deselectFlash();
  
  if(dump & 0x01) return true;
  return false;
  
}//end of FlashBusy


/* ---------------------------------------------------
 *
 *      Flash chip erase
 *
 * ---------------------------------------------------*/
void FlashErase()
{
  Flash_Write_Enable();
  selectFlash();
  SPI.transfer(0xC7);
  deselectFlash();
  delay(300);
  Flash_Write_Disable();
  while(FlashBusy());
}


/* ---------------------------------------------------
 *
 *      Flash_read_8bit
 *
 * ---------------------------------------------------*/
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


/* ---------------------------------------------------
 *
 *      Flash_write_8bit
 *
 * ---------------------------------------------------*/
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
//      Flsh_print_mem - print all write mem
//
// ---------------------------------------------------
void Flash_print_mem()
{

  uint8_t FpmData;
  
  // print first 64 mem byte
  Serial.println("");
  Serial.println("------- ...::: START :::... -------");
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
    /*if(j==15){
      Serial.println("");
      j = 0;
    }
    else j++;*/
  }
  Serial.println("");
  Serial.println("----------- pseudo FAT ------------");
  j = 0;
  for(uint32_t i = Flash_FAT_0_addres; i < Flash_data_0_addres; i++)
  {
    FpmData = SPI.transfer(0x00);
    for(uint8_t k = 0; k < 8; k++)
    {
      Serial.print(bitRead(FpmData,k));
    }
    Serial.print(" ");
    if(j==3){
      Serial.println("");
      j = 0;
    }
    else j++;
  }
  Serial.println("--------------- dane --------------");

  uint8_t dane_odczytane;
  dane_odczytane = SPI.transfer(0x00);
  while(dane_odczytane != 0xFF){
    Serial.write(dane_odczytane);
    dane_odczytane = SPI.transfer(0x00);
  }
  Serial.println("-------- ...::: END :::... --------");
  
  deselectFlash();

  // print all write memory
  
  
  
}//end of Flsh_print_mem


/* ---------------------------------------------------
 *
 *      Flash_write_string
 *      zapisuje pod wskazany adres flash ciąg
 *      i zwraca nowy dres flash
 *
 * ---------------------------------------------------*/
uint32_t Flash_write_string(uint32_t addr, String str){
  
  while(FlashBusy());
  for(int i = 0; i < str.length(); i++){  //write str to Flash
    Flash_write_8bit(addr + i, str.charAt(i));
  }
  addr = addr + str.length(); //przelicz adres
  return addr;
  
}//end of Flash_write_string


/* ---------------------------------------------------
 *
 *      
 *      
 *      
 *
 * ---------------------------------------------------*/
uint32_t Flash_find_start_address(){
  uint32_t i;
  i = Flash_data_0_addres;
  while(Flash_read_8bit(i)!=0xFF){
    i++;
  }
  return i;
}//end of Flash_find_start_address


/* ---------------------------------------------------
 *
 *     
 *      
 *      
 *
 * ---------------------------------------------------*/
uint8_t find_file_num(){

  uint8_t return_val;
  uint8_t ffnData;
  
  for(uint8_t ffnAddr = Flash_FAT_0_addres; ffnAddr < Flash_data_0_addres; ffnAddr++){
    ffnData = Flash_read_8bit(ffnAddr);
    for(uint8_t ffnBit = 0; ffnBit < 8; ffnBit++){
       if(bitRead(ffnData,ffnBit)==1){
        
          /*
           * Serial.print("address: ");
          Serial.print(ffnAddr);
          Serial.print("; bit num: ");
          Serial.print(ffnBit,HEX);
          Serial.print("; ffnData:");
          Serial.print(ffnData,BIN);
          */
          
          bitWrite(ffnData,ffnBit,0);
          
          /*
          Serial.print("; ffnData return:");
          Serial.println(ffnData,BIN);
          */
          
          Flash_write_8bit(ffnAddr, ffnData);
          
          return_val =  ffnAddr * 8 + ffnBit + 1;
          
          return return_val;
       }
    }
  }
  return 0; //
  
}//end of find_file_num
