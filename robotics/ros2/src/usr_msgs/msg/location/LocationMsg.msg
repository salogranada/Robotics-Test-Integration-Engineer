# Message containing the measurements for the IMU and GPS sensor
# roll [float32]: roll measure by IMU
# pitch [float32]: pitch measure by IMU
# yaw [float32]: yaw measure by IMU
# latitude [string]: given latitude provided by the current GNSS device
# longitude [string]: given longitude provided by the current GNSS device 
# sensor [string]: GNSS source of information GPS/WIFI

float32 roll
float32 pitch
float32 yaw
string latitude
string longitude
string sensor