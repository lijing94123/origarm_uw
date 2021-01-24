#include "ros/ros.h"
#include "origarm_uw/Command_ABL.h"
#include "origarm_uw/Seg_ABL.h"
#include "origarm_uw/Command_Position.h"
#include "origarm_uw/keynumber.h"
#include "origarm_uw/modenumber.h"
#include "origarm_uw/segnumber.h"
#include <ros/time.h>
#include "ros/package.h"

#include <linux/input-event-codes.h>
#include "myData.h"

// send command to arduino
int Cmd_Arduino[34];



