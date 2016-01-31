/* 
 * SubPos Hardware Receiver (http://www.subpos.org)
 * Copyright (C) 2016 Blair Wyatt
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

//Issues compiling on Teensy: https://forum.pjrc.com/threads/28946-Issue-compiling-for-3-1-works-fine-for-teensy-3-0

#include <Eigen30.h>
#include <math.h>
#include "extern.h"

#include "Trilateration.h"

//Set this to the hardware serial port you wish to use
#define HWSERIAL Serial1
#define D_R (M_PI / 180.0)
#define R_D (180.0 / M_PI)
#define R_MAJOR 6378137.0
#define R_MINOR 6356752.3142
#define RATIO (R_MINOR/R_MAJOR)
#define ECCENT (sqrt(1.0 - (RATIO * RATIO)))
#define COM (0.5 * ECCENT)

struct sps_data {
    uint32_t      dev_id;      //24 bit
    int32_t       lat;         //32 bit
    int32_t       lng;         //32 bit
    int32_t       altitude;    //26 bit (with extra sign bit)
    int16_t       tx_pwr;      //11 bit
    bool          off_map;     //1  bit
    bool          three_d_map; //1  bit
    uint16_t      res;         //12 bit
    uint32_t      app_id;      //24 bit
    uint8_t       path_loss;   //3  bit
};

unsigned char incomingByte;
char temp_array[5];
unsigned char subpos_array[31];
struct sps_data decoded_data; 
int i = 0;
int num_scan;
int channel;
int rssi;
unsigned int hex_value;
double distance;
int ledPin = 13;
long fixLedTimer = 0;
int nodes;

Trilateration worker;
PosAndDistance2dVec beacons;
Pos2d location;

int clockCounter = 0;

char nmea[62] = {0}; //61 chars Plus null terminator
  

/*
 * Mercator transformation
 * accounts for the fact that the earth is not a sphere, but a spheroid
 * wiki.openstreetmap.org/wiki/Mercator
 */

double deg_rad (double ang) {
    return ang * D_R;
}
 
double merc_x (double lon) {
    return R_MAJOR * deg_rad (lon);
}
 
double merc_y (double lat) {
    lat = fmin (89.5, fmax (lat, -89.5));
    double phi = deg_rad(lat);
    double sinphi = sin(phi);
    double con = ECCENT * sinphi;
    con = pow((1.0 - con) / (1.0 + con), COM);
    double ts = tan(0.5 * (M_PI * 0.5 - phi)) / con;
    return 0 - R_MAJOR * log(ts);
}
 
double rad_deg (double ang) {
    return ang * R_D;
}
 
double merc_lon (double x) {
    return rad_deg(x) / R_MAJOR;
}
 
double merc_lat (double y) {
    double ts = exp ( -y / R_MAJOR);
    double phi = M_PI_2 - 2 * atan(ts);
    double dphi = 1.0;
    int i;
    for (i = 0; fabs(dphi) > 0.000000001 && i < 15; i++) {
        double con = ECCENT * sin (phi);
        dphi = M_PI_2 - 2 * atan (ts * pow((1.0 - con) / (1.0 + con), COM)) - phi;
        phi += dphi;
    }
    return rad_deg (phi);
}


struct sps_data decode_ssid(unsigned char* str_decode){
  
    //Make string "safe" 
    uint8_t ssid[31] = {}; //SSID can be 32 octets, but we will ignore 
                           //the last octet as some embedded systems
                           //don't implement it
    memcpy(ssid, str_decode, 31);
    
    struct sps_data decoded_data;
    
  
    //Check coding bits and reconstruct data
    //we don't have to extract and check the coding mask bits
    //if we work from the right
      
    int x;
    int y = 0;
  
    for (x = 30; x >= 24; x--)
    {
        if (((ssid[30] >> y) & 0x1) == 1)
           ssid[x] = ssid[x] - 1;
        y++;
    }
      
    y = 0;
    for (x = 23; x >= 17; x--)
    {
        if (((ssid[29] >> y) & 0x1) == 1)
            ssid[x] = ssid[x] - 1;
        y++;
    }
    
    y = 0;
    for (x = 16; x >= 10; x--)
    {
        if (((ssid[28] >> y) & 0x1) == 1)
            ssid[x] = ssid[x] - 1;
        y++;
    }
    
    y = 0;
    for (x = 9; x >= 3; x--)
    {
        if (((ssid[27] >> y) & 0x1) == 1)
            ssid[x] = ssid[x] - 1;
        y++;
    }
      
    //Now pull out the "ASCII" mask
    y = 0;
    for (x = 23; x >= 17; x--)
    {
        if (((ssid[26] >> y) & 0x1) == 1)
            ssid[x] = ssid[x] | 0x80;
        y++;
    }
      
    y = 0;
    for (x = 16; x >= 10; x--)
    {
        if (((ssid[25] >> y) & 0x1) == 1)
            ssid[x] = ssid[x] | 0x80;
        y++;
    }
      
    y = 0;
    for (x = 9; x >= 3; x--)
    {
        if (((ssid[24] >> y) & 0x1) == 1)
            ssid[x] = ssid[x] | 0x80;
        y++;
    }
    
    //Now we can easily populate the struct
  
    decoded_data.dev_id       = (unsigned long)ssid[ 3] << 16 | (unsigned long)ssid[ 4] <<  8 | (unsigned long)ssid[ 5];
    decoded_data.lat          = (unsigned long)ssid[ 6] << 24 | (unsigned long)ssid[ 7] << 16 | (unsigned long)ssid[ 8] <<  8 | (unsigned long)ssid[ 9];
    decoded_data.lng          = (unsigned long)ssid[10] << 24 | (unsigned long)ssid[11] << 16 | (unsigned long)ssid[12] <<  8 | (unsigned long)ssid[13];
    decoded_data.app_id       = (unsigned long)ssid[14] << 16 | (unsigned long)ssid[15] <<  8 | (unsigned long)ssid[16];  
    decoded_data.altitude     = (unsigned long)ssid[17] << 18 | (unsigned long)ssid[18] << 10 | (unsigned long)ssid[19] <<  2 | (((unsigned long)ssid[20] >> 6) & 0x03);
    if ((((unsigned long)ssid[20] & 0x20) >> 5) & 1) decoded_data.altitude = (decoded_data.altitude * -1);
    decoded_data.off_map      = (((unsigned long)ssid[20] & 0x10) >> 4) & 1;
    decoded_data.three_d_map  = (((unsigned long)ssid[20] & 0x08) >> 3) & 1;
    decoded_data.tx_pwr       = (((unsigned long)ssid[20] & 0x07) << 8) | (unsigned long)ssid[21];
    decoded_data.tx_pwr       = decoded_data.tx_pwr - 1000;
    decoded_data.path_loss    = ((unsigned long)ssid[22] & 0xE0) >> 5;  
    decoded_data.res          = ((unsigned long)ssid[22]  & 0x1F  << 8) | (unsigned long)ssid[23];
    
    return decoded_data;
  
};

double distance_calc(int coefficient, double rssi, double tx_pwr, int channel) {
    const double c = 299.792458; //We will use MHz for freq.
    const double pi =  3.1415926535;
  
    int freq;
    switch (channel)
    {
      case 1: freq = 2412; break;
      case 2: freq = 2417; break;
      case 3: freq = 2422; break;
      case 4: freq = 2427; break;
      case 5: freq = 2432; break;
      case 6: freq = 2437; break;
      case 7: freq = 2442; break;
      case 8: freq = 2447; break;
      case 9: freq = 2452; break;
      case 10: freq = 2457; break;
      case 11: freq = 2462; break;
      case 12: freq = 2467; break;
      case 13: freq = 2472; break;
      case 14: freq = 2484; break;
      default: freq = 2412; break;
      
    }

    double mu;
    switch (coefficient) {
        case 0: mu = 1.0; break;
        case 1: mu = 2.0; break;
        case 2: mu = 2.5; break;
        case 3: mu = 3.0; break;
        case 4: mu = 3.5; break;
        case 5: mu = 4.0; break;
        case 6: mu = 4.5; break;
        case 7: mu = 5.0; break;
        default: mu = 1.0; break;
    }
    double distance = pow(10.0,(((double)(tx_pwr/10)) - rssi - 10*log10(4*pi/(c/freq)))/(20*mu));

    return distance;

}

int checksum(char *s) {
    //just an XOR of all the bytes between the $ and the *

    int c = 0;
 
    while(*s)
        c ^= *s++;
 
    return c;
}

void nmea_generate(double latitude, double longitude, double altitude, int fix, int nodes) {
    /*GGA - essential fix data which provide 3D location and accuracy data.
    
    $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    
    Where:
     GGA          Global Positioning System Fix Data
     123519       Fix taken at 12:35:19 UTC
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     1            Fix quality: 0 = invalid
                               1 = GPS fix (SPS)
                               2 = DGPS fix
                               3 = PPS fix
			                   4 = Real Time Kinematic
			                   5 = Float RTK
                               6 = estimated (dead reckoning) (2.3 feature)
			                   7 = Manual input mode
			                   8 = Simulation mode
     08           Number of satellites being tracked
     0.9          Horizontal dilution of position
     545.4,M      Altitude, Meters, above mean sea level
     46.9,M       Height of geoid (mean sea level) above WGS84
                      ellipsoid
     (empty field) time in seconds since last DGPS update
     (empty field) DGPS station ID number
     *47          the checksum data, always begins with *

     */  

    char latDirection;
    if (latitude >= 0) {latDirection = 'N';} else {latDirection = 'S'; latitude = latitude * -1;}
    char longDirection; 
    if (longitude >= 0) {longDirection = 'E';} else {longDirection = 'W'; longitude = longitude * -1;}
    
    //Fixed-point arithmetic (without floating point).
    int latDegrees = (int)latitude;
    int latMinutes = 60 * (latitude-latDegrees);
    int latMinutesDec = (60 * (latitude-latDegrees) - latMinutes) * 1000;

    int longDegrees = (int)longitude;
    int longMinutes = 60 * (longitude-longDegrees);
    int longMinutesDec = (60 * (longitude-longDegrees) - longMinutes) * 1000;
  
    sprintf(nmea, "GPGGA,%06d,%02d%02d.%03d,%c,%03d%02d.%03d,%c,%d,%02d,0.0,%03d.%01d,M,00.0,M,,",clockCounter, latDegrees,latMinutes,latMinutesDec,latDirection,longDegrees,longMinutes,longMinutesDec,longDirection,fix,nodes, (int)altitude/100, (int)((altitude/100 - (int)altitude/100)*10));
   
    Serial.print("$");
    Serial.print(nmea);
    Serial.print("*");
    Serial.print(checksum(nmea),HEX);
    Serial.println("");
    clockCounter++;
    if (clockCounter > 999999)
    {
        clockCounter = 0;
    }
}


void loop() {

    beacons.clear();
  
    temp_array[0] = 0;
    temp_array[1] = 0;
    temp_array[2] = 0;
    temp_array[3] = 0;
    temp_array[4] = 0;
  
    if (HWSERIAL.available() > 0) 
    {
        incomingByte = HWSERIAL.read();
        if (incomingByte == 'S')
        {
            //Flush \r\n
            while (HWSERIAL.available() == 0) {}
            incomingByte = HWSERIAL.read(); 
            if (incomingByte == '\r') 
            {
              
                while (HWSERIAL.available() == 0) {}
                incomingByte = HWSERIAL.read();
      
                if (incomingByte == '\n') 
                {
                    //Serial.println("");
                    //Serial.println("Start Byte");
                 
                    incomingByte = 0;
                    i = 0;
                    
                    while (incomingByte != 0x0D && i < 3)
                    {
                        while (HWSERIAL.available() == 0) {}
                        incomingByte = HWSERIAL.read();
                        temp_array[i] = incomingByte;
                        i++;
                    }
                      
                    num_scan = atoi(temp_array);
                    nodes = num_scan;
        
                    while (num_scan != 0) 
                    {
                  
                        //Flush /n (/r already flushed above)
                        while (HWSERIAL.available() < 1) {}
                        incomingByte = HWSERIAL.read();
                
                        i = 0;
                        temp_array[0] = 0;
                        temp_array[1] = 0;
                        temp_array[2] = 0;
                        temp_array[3] = 0;
                        temp_array[4] = 0;
                        incomingByte = 0;
                        
                        while (i < 31)
                        {
                            while (HWSERIAL.available() < 2) {}
                            temp_array[0] = HWSERIAL.read();
                            temp_array[1] = HWSERIAL.read();
                          
                            hex_value = (unsigned int)strtoul(temp_array, NULL, 16);
                            subpos_array[i] = hex_value;
          
                            //Serial.print(hex_value, HEX);
                            i++;
                        }
                        
                        //wait until comma
                        i=0;
                        incomingByte = 0;
                        
                        while (incomingByte != 0x2C && i < 2)
                        {
                            while (HWSERIAL.available() == 0) {}
                            incomingByte = HWSERIAL.read();
                            i++;
                        }
            
                        temp_array[0] = 0;
                        temp_array[1] = 0;
                        temp_array[2] = 0;
                        temp_array[3] = 0;
                        temp_array[4] = 0;
                        
                        //Get RSSI
                        i=0;
                        incomingByte = 0;
                        
                        while (incomingByte != 0x2C && i < 6)
                        {
                            while (HWSERIAL.available() == 0) {}
                            incomingByte = HWSERIAL.read();
                            temp_array[i] = incomingByte;
                            i++;
                        }
                        
                        rssi = atoi(temp_array);
                        rssi = rssi - 65536;
            
                        //Read Channel           
                        temp_array[0] = 0;
                        temp_array[1] = 0;
                        temp_array[2] = 0;
                        temp_array[3] = 0;
                        temp_array[4] = 0;
                        i=0;
                        incomingByte = 0;
                        
                        while (incomingByte != 0x0D && i < 2)
                        {
                            while (HWSERIAL.available() == 0) {}
                            incomingByte = HWSERIAL.read();
                            temp_array[i] = incomingByte;
                            i++;
                        }
          
                        channel = atoi(temp_array);
          
                        //SubPos SSID decode
                        decoded_data = decode_ssid(subpos_array);
                        distance = (distance_calc(decoded_data.path_loss,(double)rssi,(double)decoded_data.tx_pwr/10, channel));
          
                        num_scan--;
          
                        beacons.push_back(PosAndDistance2d(Pos2d(merc_y((double)decoded_data.lat/10000000), merc_x((double)decoded_data.lng/10000000)), distance)); 
        
                    }
        
                    //Check if there is only one or two beacons
                    if (beacons.size() == 1) 
                    {
        
                        //Just use Node position
                        nmea_generate((double)decoded_data.lat/10000000, (double)decoded_data.lng/10000000, decoded_data.altitude, 1, nodes);
        
                        //Serial.println(decoded_data.lat, DEC);
                        //Serial.println(decoded_data.lng, DEC);
        
                    }
                    else if (beacons.size() == 2) 
                    {
                        //Add these two beacons again for trilat calcs to work.
                        beacons.push_back(PosAndDistance2d(Pos2d(beacons[0].m_pos(0), beacons[0].m_pos(1)), beacons[0].m_distance));
                        beacons.push_back(PosAndDistance2d(Pos2d(beacons[1].m_pos(0), beacons[0].m_pos(1)), beacons[1].m_distance));
                        
                          
                        worker.CalculateLocation2d(beacons, location); 
                        nmea_generate(merc_lat(location[0]), merc_lon(location[1]), decoded_data.altitude, 1, nodes);
          
                        //Serial.println(merc_lat(location[0]), DEC);
                        //Serial.println(merc_lon(location[1]), DEC);
                    } 
                    else 
                    {
                        worker.CalculateLocation2d(beacons, location); 
                        nmea_generate(merc_lat(location[0]), merc_lon(location[1]), decoded_data.altitude, 1, nodes);
          
                        //Serial.println(merc_lat(location[0]), DEC);
                        //Serial.println(merc_lon(location[1]), DEC);
                    
                    }
                }
            }
        }
  
      //Serial.print(incomingByte, HEX);
      //HWSERIAL.print("UART received:");
      //HWSERIAL.println(incomingByte, DEC);
    } 
    
    //Blink LED on position lock
    if (beacons.size() < 1) 
    {  
        if (fixLedTimer == 0)
        {
            digitalWrite(ledPin, LOW); 
        } 
        if (fixLedTimer >= 0)
        {
            fixLedTimer--;
        }
        
    } else {
        fixLedTimer = 10000;
        digitalWrite(ledPin, HIGH);
    }

};

void setup() 
{
  
    //Setup serial port
    Serial.begin(9600);
    HWSERIAL.begin(115200);
    pinMode(ledPin,OUTPUT);
  
};
