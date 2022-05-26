# Message containing the measurements for the IMU and GPS sensor
# header [std_msgs/Header]: standard header message
# heading [uint16]: robot orientation
# status [int8]: fixed status
# roll [float32]: roll measure by IMU
# pitch [float32]: pitch measure by IMU
# yaw [float32]: yaw measure by IMU
# latitude [string]: given latitude provided by the current GNSS device
# longitude [string]: given longitude provided by the current GNSS device 
# accuracy [float32]: accuracy calculated by the covariance provided by GNSS device
# sat [int8]: number of satellites availables for the GNSS device
# sensor [string]: GNSS source of information GPS/WIFI

std_msgs/Header header
uint16 heading
int8 status
float32 roll
float32 pitch
float32 yaw
string latitude
string longitude
float32 accuracy
int8 sat
string sensor