# $GPGGA,153224.000,5250.7544,N,00501.9356,E,1,08,1.1,12.1,M,46.8,M,,0000*62

#1    = UTC of Position
float32 Time
#2    = Latitude
float32 Latitude
#3    = N or S
uint8 NS
#4    = Longitude
float32 Longitude
#5    = E or W
uint8 EW
#6    = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
int32 Quality
#7    = Number of satellites in use [not those in view]
int32 Sattelites
#8    = Horizontal dilution of position
float32 Dilution
#9    = Antenna altitude above/below mean sea level (geoid)
float32 Altitude
#10   = Meters  (Antenna height unit)
#string Meter
#11   = Geoidal separation (Diff. between WGS-84 earth ellipsoid and
#       mean sea level.  -=geoid is below WGS-84 ellipsoid)
float32 Separation
#12   = Meters  (Units of geoidal separation)
#string MetersGeo
#13   = Age in seconds since last update from diff. reference station
float32 AgeBase
#14   = Diff. reference station ID#
float32 DiffBase

