/*  
 * 
 * Copyright © 2023 DTU,
 * Author:
 * Christian Andersen jcan@dtu.dk
 * 
 * The MIT License (MIT)  https://mit-license.org/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies 
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE. */

#include <string>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include "mpose.h"
#include "steensy.h"
#include "uservice.h"
#include "sencoder.h"
#include "utime.h"
#include "cmotor.h"
#include "cservo.h"
#include "medge.h"
#include "cedge.h"
#include "cmixer.h"


#include "bplan100.h"

// create class object
BPlan100 plan100;


void BPlan100::setup()
{ // ensure there is default values in ini-file
  if (not ini["plan100"].has("log"))
  { // no data yet, so generate some default values
    ini["plan100"]["log"] = "true";
    ini["plan100"]["run"] = "false";
    ini["plan100"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["plan100"]["print"] == "true";
  //
  if (ini["plan100"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_plan100.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission plan100 logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

BPlan100::~BPlan100()
{
  terminate();
}


void BPlan100::run()
{
  if (not setupDone)
    setup();
  if (ini["plan100"]["run"] == "false")
    return;
  //
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 10;
  oldstate = state;
  //
  toLog("Plan100 started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    { // make a shift in heading-mission
      case 10:
        toLog("Reset pose");
        pose.resetPose();

        toLog("forward at 0.3m/s");
        mixer.setVelocity(0.3);
        state = 11;
        break;
      case 11: // wait for distance
        if (pose.dist >= 0.3)
        { // done, and then
          toLog("now turn at 0.5 rad/s and 0 m/s");
          // reset turned angle
          pose.turned = 0.0;
          mixer.setVelocity(0.0);
          mixer.setTurnrate(0.5);
          state = 21;
        }
        else if (t.getTimePassed() > 10)
          lost = true;
        break;
      case 21:
        if (pose.turned >= M_PI)
        {
          mixer.setDesiredHeading(M_PI);
          toLog("now go back");
          mixer.setVelocity(0.3);
          // reset driven distance
          pose.dist = 0;
          state = 31;
        }
        else if (t.getTimePassed() > 12)
          lost = true;
        break;
      case 31: // wait for distance
        if (pose.dist >= 0.3)
        { // the end
          mixer.setVelocity(0.0);
          finished = true;
        }
        else if (t.getTimePassed() > 10)
          lost = true;
        break;
      default:
        toLog("Unknown state");
        lost = true;
        break;
    }
    if (state != oldstate)
    {
      oldstate = state;
      toLog("state start");
      // reset time in new state
      t.now();
    }
    // wait a bit to offload CPU
    usleep(2000);
  }
  if (lost)
  { // there may be better options, but for now - stop
    toLog("Plan100 got lost");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("Plan100 finished");
}


void BPlan100::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BPlan100::toLog(const char* message)
{
  UTime t("now");
  if (logfile != nullptr)
  {
    fprintf(logfile, "%lu.%04ld %d %% %s\n", t.getSec(), t.getMicrosec()/100,
            oldstate,
            message);
  }
  if (toConsole)
  {
    printf("%lu.%04ld %d %% %s\n", t.getSec(), t.getMicrosec()/100,
           oldstate,
           message);
  }
}

