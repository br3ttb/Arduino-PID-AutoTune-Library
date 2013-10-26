// enumerated types used in PID and PID_ATune classes

#ifndef PID_TYPES_H
#define PID_TYPES_H

enum Mode
{
  MANUAL = 0,
  AUTOMATIC = 1
};

enum Direction
{
  DIRECT = 1,
  REVERSE = -1
};

enum Control
{
  ZIEGLER_NICHOLS_PI = 0,	
  ZIEGLER_NICHOLS_PID = 1,
  PESSEN = 2,
  SOME_OVERSHOOT = 3,
  NO_OVERSHOOT = 4
};

enum Peak
{
  MINIMUM = -1,
  NON_PEAK = 0,
  MAXIMUM = 1
};

#endif

