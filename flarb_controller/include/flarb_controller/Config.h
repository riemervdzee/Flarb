#ifndef CONFIG_H
#define CONFIG_H


// Extra radius of the Flarb vehicle
#define FLARB_EXTRA_RADIUS 0.20f

// cAvoidObstacle
#define FLARB_AVOID_WAITTIME      3
#define FLARB_AVOID_SPEED     0.25f
#define FLARB_AVOID_GOALANGLE 0.12f // The tollerance of the goal angle, in rad

// cSegmentFollow
#define FLARB_FOLLOW_EXTRA    0.02f // Attempt to get an extra radius, so we are more in the middle of the path
#define FLARB_FOLLOW_SPEED    0.50f

// cSegmentFind
#define FLARB_FIND_SPEED      0.35f
#define FLARB_FIND_SPEEDANGLE 1.05f // The angle we are turning at
#define FLARB_FIND_GOALANGLE  0.12f // The tollerance of the goal angle, in rad


#endif // CONFIG_H
