/* 
 * File:   Timer.h
 * Author: matt
 *
 * Created on 05 November 2012, 10:19
 */

#ifndef TIMER_H
#define	TIMER_H

#include <signal.h>
#include <iostream>

#include "MPU6050.h"
#include "AHRS.h"
#include "Logger.h"
#include "PICInterface.h"
#include "Control.h"

class TimerClass
{
public:
    TimerClass();
    TimerClass(const TimerClass& orig);
    virtual ~TimerClass();
    
    void start();
    void stop();
    
    float dt;
    timer_t timerId; //id  for the posix timer
    struct sigaction signalAction; //signal action handler struct
    bool started;
private:
    static void sig_handler_(int signum);
    struct itimerspec timeToSet_; //time to be set
    struct timespec timeValue_; //timer expiration value
    struct timespec timeInterval_; //timer period
    timespec oldtime_;
    timespec time_;
    timespec iterationtime_;
    
    void calcdt_();
    void compensate_();

};

extern TimerClass Timer;
#endif	/* TIMER_H */

