#define     FWV                     ("ATC 200 firmware version: 2.2.0")


/////////Inputs (switches are normally close)/////////

//Telecover Photo sensors
#define     POS_SENSOR_CW           12 // 25 ( Atmel pin )     
#define     POS_SENSOR_CCW          4  // 26 ( Atmel pin )    

//Telearm switches
#define     TELECOVER_ENABLED       10 // 29 ( Atmel pin )
#define     TELECOVER_DISABLED      9  // 30 ( Atmel pin )
    
    
#define     V_read                  A0 // 36 ( Atmel pin )
    
    
/////////////Outputs///////////////   
    
#define     MOVE_CW                 3  // 19 ( Atmel pin )
#define     MOVE_CCW                2  // 18 ( Atmel pin )
                                       
#define     MOVE_IN                 8  // 28 ( Atmel pin )
#define     MOVE_OUT                6  // 27 ( Atmel pin )
                                       
/////////PWM///////////                                       
#define     PWM                     11 // 12 ( Atmel pin )

/////////Telecover Positions//////////
#define     NORTH                   1
#define     EAST                    2
#define     SOUTH                   3
#define     WEST                    4

////////Move Type (Clockwise or counter clockwise//////        
#define     CCLOCKW                 0
#define     CLOCKW                  1
        
///////Arm Motor Status/////////
#define     Close                   0 
#define     Open                    1
#define     Moving                  2
#define     Time_Out                3
#define     Not_Connected           4

///////Disk Motor Status/////////

#define     Moving                  2
#define     Time_Out                3
#define     Not_Connected           4
#define     In_Position             5


//////Speed/////
#define     MAX_SPEED_ARM           185
#define     MIN_SPEED_DISK_CW       110
#define     MIN_SPEED_DISK_CCW      185
#define     MAX_SPEED_DISK_CW       220
#define     MAX_SPEED_DISK_CCW      50
#define     MIN_SPEED_ARM           110
#define     DEFAULT_ARM_SPEED       100
#define     DEFAULT_DISK_SPEED_CW   150
#define     DEFAULT_DISK_SPEED_CCW  100

/////Timeout/////
#define     DEFAULT_TIMEOUT         10000

#define     ENABLED                 1   // Telecover enabled
#define     DISABLED                0   // Telecover disabled


///////Commands///////
#define ARM_SPEED       'A'
#define DISK_SPEED_CW   'K'
#define DISK_SPEED_CCW  'L'
#define TIMEOUT         'T'
#define SERVICE_MODE    'M'
#define VERSION         'V'
#define MOVE_NORTH      'N'
#define MOVE_EAST       'E'
#define MOVE_SOUTH      'S'
#define MOVE_WEST       'W'
#define CLOSE           'C'
#define OPEN            'O'
#define GO_HOME         'H'
#define DEMO            'D'
#define SHOW_STATUS     'P'
 
