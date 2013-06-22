#ifndef CONFIG_H
#define CONFIG_H

// Extra radius of the Flarb vehicle
#define FLARB_EXTRA_RADIUS 0.22f

// cAvoidObstacle
#define FLARB_AVOID_WAITTIME      3
#define FLARB_AVOID_SPEED     0.25f
#define FLARB_AVOID_GOALANGLE 0.12f // The tollerance of the goal angle, in rad

// cSegmentFind
#define FLARB_FIND_SPEED      0.35f
#define FLARB_FIND_SPEEDANGLE 1.05f // The angle we are turning at
#define FLARB_FIND_GOALANGLE  0.12f // The tollerance of the goal angle, in rad

#endif // CONFIG_H
