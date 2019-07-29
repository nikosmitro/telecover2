//Raymetrics TLC400 firmware
//Release version: 0.1.4
//Release date: 13-05-2016
//Artist: Harris Triantafillidis
//Raymetrics S.A. all rights reserved

#include <EEPROM.h>
#include "IO.h"
#include <avr/eeprom.h>


//***********************  GLOBAL VARIABLES **********************//

char            S_CMD;
char            DiskPosition[6];

uint8_t         Counter;

uint8_t         sensorToCount, sensorToEnd;

bool            test_mode = false,

                to_move = false,
                clockwise = true,
                ArmTimeOut = false,
                DiskTimeOut = false;

uint16_t         Error = 0, Previous_Error = 0, V_temp = 0,
                 Speed = 140, lastSpeed = 0,
                 ArmSpeed = 100, DiskSpeed_CW = 100, DiskSpeed_CCW = 100, BrakeSpeed;

unsigned long   LastReset;

float           Vin = 0;

uint16_t        TimeOut = 10000;

byte            IsArmPresent, IsDiskPresent;
byte            ArmStatus, DiskStatus;
//Status -> 0:Close, 1:Open, 2:Moving, 3:Timeout, 4:Not connected



//**** variables in eeprom  ********//

int16_t     EEMEM     eeArmSpeed;
int16_t     EEMEM     eeDiskSpeedCW;
int16_t     EEMEM     eeDiskSpeedCCW;
uint16_t    EEMEM     eeTimeOut;
byte        EEMEM     eeIsArmPresent;
byte        EEMEM     eeIsDiskPresent;
uint8_t     EEMEM     eeCounter;



//********FUNCTIONS********//
bool getIfClockwise(uint8_t* start, uint8_t* destination);
bool goHome();
bool moveTelearm(uint8_t target_pos);
void stopRotate();
bool moveTelecover(uint8_t target_pos);
void V_input();
void get_status();
void print_status();
void incoming_data();
void writePresentMotors();
void readPresentMotors();



void setup()
{
  Serial.begin(9600);

  //Inputs setup
  pinMode(POS_SENSOR_CW, INPUT);
  pinMode(POS_SENSOR_CCW, INPUT);
  digitalWrite(POS_SENSOR_CW, LOW);
  digitalWrite(POS_SENSOR_CCW, LOW);
  pinMode(TELECOVER_ENABLED, INPUT);
  pinMode(TELECOVER_DISABLED, INPUT);
  digitalWrite(TELECOVER_ENABLED, LOW);
  digitalWrite(TELECOVER_DISABLED, LOW);

  //Outputs setup
  pinMode(MOVE_CW, OUTPUT);
  pinMode(MOVE_CCW, OUTPUT);
  digitalWrite(MOVE_CW, LOW);
  digitalWrite(MOVE_CCW, LOW);
  pinMode(MOVE_IN, OUTPUT);
  pinMode(MOVE_OUT, OUTPUT);
  digitalWrite(MOVE_IN, LOW);
  digitalWrite(MOVE_OUT, LOW);


  pinMode(PWM, OUTPUT);
  digitalWrite(PWM, LOW);
  //analogWrite(PWM, Speed);

  ArmSpeed = eeprom_read_word((uint16_t *) &eeArmSpeed);
  //  Speed=EEPROM.read(0);
  if (ArmSpeed > MAX_SPEED_ARM || ArmSpeed < MIN_SPEED_ARM)      //if EEPROM value out of limits, set Speed to default
  {
    ArmSpeed = DEFAULT_ARM_SPEED;
    eeprom_write_word((uint16_t *)&eeArmSpeed, ArmSpeed);
  }

  DiskSpeed_CW = eeprom_read_word((uint16_t *) &eeDiskSpeedCW);
  //  Speed=EEPROM.read(0);
  if (DiskSpeed_CW > MAX_SPEED_DISK_CW || DiskSpeed_CW < MIN_SPEED_DISK_CW)      //if EEPROM value out of limits, set Speed to default
  {
    DiskSpeed_CW = DEFAULT_DISK_SPEED_CW;
    eeprom_write_word((uint16_t *)&eeDiskSpeedCW, DiskSpeed_CW);
  }


  DiskSpeed_CCW = eeprom_read_word((uint16_t *) &eeDiskSpeedCCW);
  //  Speed=EEPROM.read(0);
  if (DiskSpeed_CCW < MAX_SPEED_DISK_CCW || DiskSpeed_CCW > MIN_SPEED_DISK_CCW)      //if EEPROM value out of limits, set Speed to default
  {
    DiskSpeed_CCW = DEFAULT_DISK_SPEED_CCW;
    eeprom_write_word((uint16_t *)&eeDiskSpeedCCW, DiskSpeed_CCW);
  }
  
  TimeOut = eeprom_read_word((uint16_t *) &eeTimeOut);
  if (TimeOut <= 0 || TimeOut > 50000)
  {
    TimeOut = DEFAULT_TIMEOUT;
    eeprom_write_word((uint16_t *)&eeTimeOut, TimeOut);
  }
  
  Counter = eeprom_read_byte((uint8_t *)&eeCounter);

  readPresentMotors();

  while (!Serial) {
    ;
  }

  get_status();
  print_status();
  Previous_Error = Error;
  //goHome();
}

void loop()
{
  if (Serial.available())
  {
    incoming_data();
  }

  get_status();
  if (Previous_Error != Error)
  {
    Previous_Error = Error;
    print_status();
  }
}

// AL is the best in algorithm design!!!!!!!!!!!!
// Best anti/clockwise direction calculation!!!!!!!!
bool getIfClockwise(uint8_t* start, uint8_t* destination)
{
  bool clockwise = false;
  int8_t diff = *destination - *start;
  if (diff == 1 || diff == -3)
  {
    clockwise = true;
  }
  else
  {
    clockwise = false;
  }
  return clockwise;
}

// Move telecover to home position:
// DISABLED and rotated to NORTH.
bool goHome()
{
  if (moveTelearm(DISABLED))
  {
    moveTelecover(NORTH);
    return true;
  }
  return false;
}

// Moves the telecover in or out of position:
// ENABLED means move the telecover over the telescope,
// DISABLED means move it to the side.
// Returns a boolean indicating if the command succeeded.
bool moveTelearm(uint8_t target_pos)
{
  if (target_pos != 0 && target_pos != 1)
    return false;
  uint8_t pinToWriteHigh = 0, pinToWriteLow = 0,
          switchToRead = 0;
  uint32_t startOfMove = millis();
  uint32_t elapsedTime = 0;
  uint32_t start_time, current, brake_time, delay_time; 
  if (target_pos == ENABLED)
  {
    if (digitalRead(TELECOVER_ENABLED) == LOW)
    {
      return true;
    }
    else
    {
      brake_time = 500;
      delay_time = 72;
      BrakeSpeed = 210 - MIN_SPEED_ARM;
      start_time = millis();
      Serial.println("CLOSE");
      switchToRead   = TELECOVER_ENABLED;
      digitalWrite(MOVE_IN, HIGH);
      analogWrite(MOVE_OUT, 210 - ArmSpeed);
    }
  }
  else if (target_pos == DISABLED)
  {
    if (digitalRead(TELECOVER_DISABLED) == LOW)
    {
      return true;
    }
    else  
    {
      brake_time = 300; 
      delay_time = 42;
      BrakeSpeed = MIN_SPEED_ARM;
      start_time = millis();
      Serial.println("OPEN");
      switchToRead   = TELECOVER_DISABLED;
      digitalWrite(MOVE_IN, LOW);
      analogWrite(MOVE_OUT,ArmSpeed);
    }
  }
  /*Serial.print("The BrakeSpeed is: ");
  Serial.println(BrakeSpeed);
  Serial.print("The Brake time is: ");
  Serial.println(brake_time);*/
  while (digitalRead(switchToRead) &&  elapsedTime < TimeOut)
  {
    //reduce the speed near the end to not damage the switch
    if((current = millis() - start_time) == brake_time)
      {
        analogWrite(MOVE_OUT, BrakeSpeed);
      }
    // Wait for telearm to move
    elapsedTime = millis() - startOfMove;
  }

  if(elapsedTime > TimeOut)
  {
    ArmTimeOut = true;
   
  }
  //Delay because we have magnet reed switch
  delay(delay_time);
  // STOP
  digitalWrite(MOVE_IN, HIGH);
  digitalWrite(MOVE_OUT, HIGH);

  uint16_t movement_time_arm = millis() - start_time;
  Serial.print("Time of Arm movement : ");
  Serial.print(movement_time_arm);
  Serial.println(" ms.");
  return (ArmTimeOut);
}


// Function that rotates telecover in the
// desired orientation, following the minimum
// distance path.
void startRotate(uint8_t target_pos)
{
  if (target_pos == Counter && (!digitalRead(POS_SENSOR_CW) && !digitalRead(POS_SENSOR_CCW)))
  {
    return true;
  }
  // Start motor, determine which sensor will do the counting
  if (getIfClockwise(&Counter, &target_pos)) // if clockwise
  {
    analogWrite(MOVE_CW, DiskSpeed_CW);
    digitalWrite(MOVE_CCW, LOW);
    sensorToCount = POS_SENSOR_CW;
    sensorToEnd   = POS_SENSOR_CCW;
    BrakeSpeed = MIN_SPEED_DISK_CW + 20;

  }
  else
  {
    analogWrite(MOVE_CW, DiskSpeed_CCW);
    digitalWrite(MOVE_CCW, HIGH);
    sensorToCount = POS_SENSOR_CCW;
    sensorToEnd   = POS_SENSOR_CW;
    BrakeSpeed = MIN_SPEED_DISK_CCW - 80;
  } 
}

void stopRotate()
{
  //Serial.print("Now it will stop.\n");
  digitalWrite(MOVE_CW, HIGH);
  digitalWrite(MOVE_CCW, HIGH);
}

bool moveTelecover(uint8_t target_pos )
{
  bool ok = 0;
  uint32_t start_time, tt;
  bool two_rounds = false;
  int count_round = 1;
  //Start rotating
  
  //check if the difference of the positions is 2, so we need to pass 1 position
  if(abs(Counter - target_pos)==2)
  {
    two_rounds = true;
  }
  //Start the moving
  startRotate(target_pos);
  start_time = millis();
  while((!ok) && ((tt = millis()-start_time) < TimeOut))
  {
   if (!digitalRead(sensorToCount))
   {
    //Serial.print("Sensor to count is in a hole \n");
    if(digitalRead(sensorToEnd))
    {
      while(!digitalRead(sensorToCount))
      { 
        //Wait to leave the current hole
      }
      Counter++;
      //Reduce the speed when we reach the desired position to be exactly in place
      if(two_rounds)
      {
        if(count_round == 3)
        {
          //Serial.println("Round Two");
          analogWrite(MOVE_CW, BrakeSpeed);
        }
      }
      else
      {
        //Serial.println("Just one Round");
        analogWrite(MOVE_CW, BrakeSpeed);
      }
    }
    else if(!digitalRead(sensorToCount) && !digitalRead(sensorToEnd))
    {
      //Serial.println("Both in the holes");
      if(Counter != target_pos)
      {
        //Serial.println("Not in the desired pos");
        Counter = 0;
        ok = 0;
        count_round++;
        while(!digitalRead(sensorToCount))
        {
          //Wait to leave the current hole
        }
      }
      else
      {
        //Serial.println("In the desired pos");
        //stop moving
        ok=1;
      }
    }
   }
  } 
  eeprom_write_byte((uint8_t *)&eeCounter, Counter);
  if(tt > TimeOut)
  {
    DiskTimeOut = 1;
  }
  stopRotate();
  uint16_t movement_time_disk = millis() - start_time;
  Serial.print("Time of Disk movement : ");
  Serial.print(movement_time_disk);
  Serial.println(" ms.");
  return ok;
} 

// Read amperage
void V_input()
{
  V_temp = analogRead(V_read);
  delay(50);
  Vin = (V_temp / 204.6) / 0.397;
}

// Update current status
void get_status()
{
  int i, j, count = 0;
  Error = B100000;
  //The last 2 bits of the Error represent each motor( last bit -> Arm Motor,
  //second from the end bit -> Disk Motor
  //If motor is not connected bit=1 , else bit=0.

  //For the Disk Motor
  if (DiskTimeOut)
  {
    DiskStatus = Time_Out;
  }
  else if (!digitalRead(POS_SENSOR_CW) && !digitalRead(POS_SENSOR_CCW))
  {
    DiskStatus = In_Position;     //in position
    //Determine The Position
    if (Counter == NORTH)   strcpy(DiskPosition , "North");//DiskPosition[6] = "North";
    if (Counter == EAST)    strcpy(DiskPosition , "East");//DiskPosition[6] = "East";
    if (Counter == SOUTH)   strcpy(DiskPosition , "South");//DiskPosition[6] = "South";
    if (Counter == WEST)    strcpy(DiskPosition , "West");//DiskPosition[6] = "West";
  }
  else
  {
    for (j = 0; j < 10; i++)
    {
      if ((digitalRead(POS_SENSOR_CW) && digitalRead(POS_SENSOR_CCW))||(digitalRead(POS_SENSOR_CW) && !digitalRead(POS_SENSOR_CCW))||(!digitalRead(POS_SENSOR_CW) && digitalRead(POS_SENSOR_CCW)))
      {
        count ++;
        delay (30);
        if (count == 10)
        {
          DiskStatus = Not_Connected;     //not_connected
          Error = Error | B00010;
          break;
        }
      }
      else
      {
        DiskStatus = Moving;     //moving
        break;
      }
    }
    count = 0;
  }
  if (ArmTimeOut)
  {
    ArmStatus = Time_Out;
  }
  else
  {
    uint8_t telearmPos = 0;
    if (!digitalRead(TELECOVER_DISABLED))
      telearmPos += 1;
    if (!digitalRead(TELECOVER_ENABLED))
      telearmPos += 2;

    switch (telearmPos)
    {
      case (0):
        ArmStatus = Not_Connected;
        Error = Error | B00001;
        break;
      case (1):
        ArmStatus = Open;
        break;
      case (2):
        ArmStatus = Close;
        break;
      case (3):
        ArmStatus = Moving;
        break;
    }
  }
  //For the Disk Motor
  V_input();
  if (Vin < 7.0 || Vin > 12.5)
  {
    Error = Error | B10000;
  }
  else
  {
    Error = Error & B01111;
  }
}


void print_status()
{
  /* if (Previous_Error 1= Error)
    {
      Previous_Error = Error;
    }
  */
  Previous_Error = Error;
  Serial.print("Error :");
  Serial.print(Error, BIN);
  Serial.print(", ");
  Serial.print("Arm Motor: ");
  Serial.print(ArmStatus);
  Serial.print(" ,   ");

  Serial.print("Disk Motor: ");
  if (DiskStatus == In_Position)
  {
    Serial.print(DiskStatus);
    Serial.print(" (Position : ");
    Serial.print(DiskPosition);
    Serial.print(").\n");
  }
  else
  {
    Serial.println(DiskStatus);
  }
}

// Read Serial port
void incoming_data()
{
  S_CMD = Serial.read();
  //S_CMD = SerialBuffer[0];
  switch (S_CMD)
  {
    case (ARM_SPEED):
      if (test_mode == true)
      {
        int16_t New_Arm_Speed = Serial.parseInt();
        if ((New_Arm_Speed != ArmSpeed) && (New_Arm_Speed <= MAX_SPEED_ARM) && (New_Arm_Speed >= MIN_SPEED_ARM))
        {
          ArmSpeed = New_Arm_Speed;
          //EEPROM.update(1 , Arm_Speed);
          eeprom_write_word((uint16_t *)&eeArmSpeed, ArmSpeed);
          Serial.print("New Arm Speed = ");
          Serial.println(ArmSpeed);
        }
        else
        {
          Serial.print("Speed Range : ");
          Serial.print(MIN_SPEED_ARM);
          Serial.print("-");
          Serial.println(MAX_SPEED_ARM);
        }
      }
      else
      {
        Serial.print("Arm Speed = ");
        Serial.println(ArmSpeed);
      }
      break;

    case (DISK_SPEED_CW):
      if (test_mode == true)
      {
        int16_t New_Disk_Speed_CW = Serial.parseInt();
        if ((New_Disk_Speed_CW != DiskSpeed_CW) && (New_Disk_Speed_CW <= MAX_SPEED_DISK_CW) && (New_Disk_Speed_CW >= MIN_SPEED_DISK_CW))
        {
          DiskSpeed_CW = New_Disk_Speed_CW;
          //EEPROM.update(1 , Arm_Speed);
          eeprom_write_word((uint16_t *)&eeDiskSpeedCW, DiskSpeed_CW);
          Serial.print("New Disk Clockwise Speed = ");
          Serial.println(DiskSpeed_CW);
        }
        else
        {
          Serial.print("Speed Range : ");
          Serial.print(MIN_SPEED_DISK_CW);
          Serial.print("-");
          Serial.println(MAX_SPEED_DISK_CW);
        }
      }
      else
      {
        Serial.print("Disk Clockwise Speed = ");
        Serial.println(DiskSpeed_CW);
      }
      break;
      
    case (DISK_SPEED_CCW):
      if (test_mode == true)
      {
        int16_t New_Disk_Speed_CCW = Serial.parseInt();
        if ((New_Disk_Speed_CCW != DiskSpeed_CCW) && (New_Disk_Speed_CCW >= MAX_SPEED_DISK_CCW) && (New_Disk_Speed_CCW <= MIN_SPEED_DISK_CCW))
        {
          DiskSpeed_CCW = New_Disk_Speed_CCW;
          //EEPROM.update(1 , Arm_Speed);
          eeprom_write_word((uint16_t *)&eeDiskSpeedCCW, DiskSpeed_CCW);
          Serial.print("New Disk CounterClockwise Speed = ");
          Serial.println(DiskSpeed_CCW);
        }
        else
        {
          Serial.print("Speed Range : ");
          Serial.print(MIN_SPEED_DISK_CCW);
          Serial.print("-");
          Serial.println(MAX_SPEED_DISK_CCW);
        }
      }
      else
      {
        Serial.print("Disk CounterClockwise Speed = ");
        Serial.println(DiskSpeed_CCW);
      }
      break;

    case (TIMEOUT):
      if (test_mode == true)
      {
        TimeOut = Serial.parseInt();
        //EEPROM.write(1 , TimeOut);
        eeprom_write_word((uint16_t *)&eeTimeOut, TimeOut);
        Serial.print("TimeOut = ");
        Serial.println(TimeOut);
      }
      else
      {
        Serial.print("Time Out = ");
        Serial.println(TimeOut);
      }
      break;

    case (SERVICE_MODE):
      if (test_mode == true)
      {
        test_mode = false;
        Serial.println("Normal Operation");
      }
      else
      {
        test_mode = true;
        Serial.println("Test Mode");
      }
      break;

    case (DEMO):
      if (test_mode == true)
      {
        Serial.println("Here");
        while (test_mode == true);
        {
          Serial.println("Here2");
          S_CMD = Serial.read();
          if (S_CMD == 'R')
          {
            test_mode = false;
          }
          if (moveTelearm(ENABLED)) moveTelecover(NORTH);
          get_status();
          print_status();
          delay(1000);
          if (moveTelearm(ENABLED)) moveTelecover(EAST);
          get_status();
          print_status();
          delay(1000);
          moveTelearm(DISABLED);
          if (moveTelearm(ENABLED)) moveTelecover(SOUTH);
          get_status();
          print_status();
          delay(1000);
          moveTelearm(DISABLED);
          if (moveTelearm(ENABLED)) moveTelecover(WEST);
          get_status();
          print_status();
          delay(1000);
          moveTelearm(DISABLED);
          if (moveTelearm(ENABLED)) moveTelecover(EAST);
          get_status();
          print_status();
          delay(1000);
          moveTelearm(DISABLED);
          get_status();
          print_status();
          delay(1000);
          moveTelearm(ENABLED);
          get_status();
          print_status();
        }
      }
      break;

    case (VERSION):
      if (test_mode == true)
      {
        Serial.print(FWV);
        Serial.println(" ( Test Mode )");
      }
      else
      {
        Serial.println(FWV);
      }
      break;

    case (MOVE_NORTH):
      if (test_mode == false)
      {
        if (moveTelearm(ENABLED))
          moveTelecover(NORTH);
        get_status();
        print_status();
      }
      break;

    case (MOVE_EAST):
      if (test_mode == false)
      {
        if (moveTelearm(ENABLED))
          moveTelecover(EAST);
        get_status();
        print_status();
      }
      break;

    case (MOVE_SOUTH):
      if (test_mode == false)
      {
        if (moveTelearm(ENABLED))
          moveTelecover(SOUTH);
        get_status();
        print_status();
      }
      break;

    case (MOVE_WEST):
      if (test_mode == false)
      {
        if (moveTelearm(ENABLED))
          moveTelecover(WEST);
        get_status();
        print_status();
      }
      break;

    case (GO_HOME):
      if (test_mode == false)
      {
        goHome();
        get_status();
        print_status();
      }
      break;

    case (CLOSE):
      moveTelearm(ENABLED);
      get_status();
      print_status();
      break;

    case (OPEN):
      moveTelearm(DISABLED);
      get_status();
      print_status();
      break;

    case (SHOW_STATUS):
      get_status();
      print_status();
      break;

  }
}

void readPresentMotors()
{
  IsArmPresent = eeprom_read_byte((uint8_t *) &eeIsArmPresent);
  IsDiskPresent = eeprom_read_byte((uint8_t *) &eeIsDiskPresent);
}

void writePresentMotors()
{
  eeprom_write_byte((uint8_t *)&eeIsArmPresent, IsArmPresent);
  eeprom_write_byte((uint8_t *)&eeIsDiskPresent, IsDiskPresent);
}
