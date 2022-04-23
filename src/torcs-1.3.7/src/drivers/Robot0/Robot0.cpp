/***************************************************************************

    file                 : Robot0.cpp
    created              : jue mar 2 10:14:51 CET 2017
    copyright            : (C) 2002 INVETT-Car

 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

// Includes para STR
#include "Fuzzy.h"
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>


static tTrack	*curTrack;

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newrace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s); 
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 

// Funciones para STR
static void accelBrakeAuto(double currentTrack, double timeToNextStretch, double currentRadius, double nextRadius, tCarElt *car);
static void controlSpeed(double targetSpeed, tCarElt* car);
static void setAcelerador(double accBrake, tCarElt* car);
static int gearAuto(tCarElt *car);

static void steerAuto(double angularError, double lateralError, int segType, double radius, tCarElt *car);
static void setSteerRadius(double radius, tCarElt* car);

static float getDistToNextStretch(tCarElt* car);
static int getNextStretchType(tCarElt* car);
static float getNextRadius(tCarElt* car);
static float getDistToSegEnd(tCarElt* car);
static float getNextStretchLength(tCarElt* car);


void error(const char *msg);

// Variables para STR
Fuzzy *mySteerFuzzy, *mySpeedFuzzy;
int sockfd = 0, newsockfd = 0, connected = 0;
int lonMode = 0, latMode = 0;


/* 
 * Module entry point  
 */ 
extern "C" int 
Robot0(tModInfo *modInfo) 
{
    memset(modInfo, 0, 10*sizeof(tModInfo));

    modInfo->name    = strdup("Robot0");		/* name of the module (short) */
    modInfo->desc    = strdup("");	/* description of the module (can be long) */
    modInfo->fctInit = InitFuncPt;		/* init function */
    modInfo->gfId    = ROB_IDENT;		/* supported framework version */
    modInfo->index   = 1;

    return 0; 
} 

/* Module interface initialization. */
static int 
InitFuncPt(int index, void *pt) 
{ 
    tRobotItf *itf  = (tRobotItf *)pt; 

    itf->rbNewTrack = initTrack; /* Give the robot the track view called */ 
				 /* for every track change or new race */ 
    itf->rbNewRace  = newrace; 	 /* Start a new race */
    itf->rbDrive    = drive;	 /* Drive during race */
    itf->rbPitCmd   = NULL;
    itf->rbEndRace  = endrace;	 /* End of the current race */
    itf->rbShutdown = shutdown;	 /* Called before the module is unloaded */
    itf->index      = index; 	 /* Index used if multiple interfaces */
    return 0; 
} 

/* Called for every track change or new race. */
static void  
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) 
{ 
    curTrack = track;
    *carParmHandle = NULL; 
} 

/* Start a new race. */
static void  
newrace(int index, tCarElt* car, tSituation *s) 
{
    mySteerFuzzy = new Fuzzy("/usr/local/share/games/torcs/Steering.fis");
    mySpeedFuzzy = new Fuzzy("/usr/local/share/games/torcs/Speed.fis");

    // Inicialización del socket
    socklen_t clilen;
    char buffer[256];
    struct sockaddr_in serv_addr, cli_addr;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");

    bzero((char *) &serv_addr, sizeof(serv_addr));

    int portno = 12321;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);

    printf("ESPERANDO CONEXION CON EL SOCKET\n");

    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
        error("ERROR on binding");
    listen(sockfd,5);
    clilen = sizeof(cli_addr);
    newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
    if (newsockfd < 0)
        error("ERROR on accept");
    bzero(buffer,256);

    // Lee y se queda atrancado hasta que le llegue el mensaje de Inicialización

    int n = read(newsockfd,buffer,255);
    if (n < 0) error("ERROR reading from socket");

    int code = 0;
    if(sscanf(buffer, "Controller init request %d", &code))
    {
        latMode = code / 10; // digito 1
        lonMode = code % 10; // digito 2
        printf("Code: %d\n", code);

        if(latMode >= 1 && latMode <= 3 && lonMode >= 1 && lonMode <= 3){
            connected = 1;
            printf("Conexion establecida correctamente\n");
            fcntl(newsockfd, F_SETFL, fcntl(newsockfd, F_GETFL, 0) | O_NONBLOCK);

            if(latMode == 1)
                printf("Control lateral automatico\n");
            if(latMode == 2)
                printf("Control lateral radio de curvatura\n");
            if(latMode == 3)
                printf("Control lateral posicion volante\n");
            if(lonMode == 1)
                printf("Control longitudinal automatico\n");
            if(lonMode == 2)
                printf("Control longitudinal velocidad\n");
            if(lonMode == 3)
                printf("Control longitudinal acelerado-freno\n");
        }
        else
        {
            connected = 0;
            printf("Not initialized: %s\n",buffer);
        }
    }

} 

/* Drive during race. */
static void drive(int index, tCarElt* car, tSituation *s) {
    memset((void *)&car->ctrl, 0, sizeof(tCarCtrl)); 
    car->ctrl.brakeCmd = 1.0; /* all brakes on ... */ 

    double angularError = RtTrackSideTgAngleL(&(car->pub.trkPos)) - car->pub.DynGC.pos.az;
    NORM_PI_PI(angularError); // put the angle back in the range from -PI to PI
    double lateralError = - car->pub.trkPos.toMiddle;
    int stretchType = car->pub.trkPos.seg->type;
    double radius = car->pub.trkPos.seg->radius;
    double distToNextStretch = getDistToNextStretch(car);
    int nextStretchType = getNextStretchType(car);
    double nextRadius = getNextRadius(car);
    double nextStretchLength = getNextStretchLength(car);
    double currentSpeed = sqrt(car->_speed_x*car->_speed_x+car->_speed_y*car->_speed_y);
    double timeToNextStretch = 0;
    if(currentSpeed == 0)
        timeToNextStretch = 10000;
    else
        timeToNextStretch = distToNextStretch / currentSpeed;


    printf("ae: %.3f le: %.3f segType: %d radius: %.2f\n", angularError, lateralError, stretchType, radius);
    printf("Dsit to next stretch: %.2f\n", distToNextStretch);
    printf("NextSegType: %d\n", nextStretchType);
    printf("NextRadius: %.2f\n", nextRadius);
    printf("NextStretchLength: %.2f\n", nextStretchLength);
    printf("Speed: %.2f\n", currentSpeed);
    printf("Time2NextStretch: %.2f\n", timeToNextStretch);

    char buffer[256];
    static float var1 , var2; // var1 "volante", var2 "velocidad"
    float auxVar1, auxVar2;
    // Lee si ha llegado una orden de control
    int n = read(newsockfd,buffer,255);
    if(n>0)
        if(sscanf(buffer, "%f;%f*", &auxVar1, &auxVar2)==2)
        {
            printf("Leidos: var1(volante) %f var2(velocidad) %f\n", var1, var2);
            var1 = auxVar1;
            var2 = auxVar2;
        }

    if(var1 != 0.0)
        printf("AQUI HA CAMBIADO: %e %s\n", var1, buffer);


    if(latMode == 1) // lat auto
        steerAuto(angularError, lateralError, stretchType, radius, car);
    else if(latMode == 2) // lat - var1 - radio
        setSteerRadius((double)var1, car);
    else if(latMode == 3) // lat - var1- volante
        car->_steerCmd = var1;

    if(lonMode == 1) // lon auto
    {
        accelBrakeAuto(stretchType, timeToNextStretch, radius, nextRadius, car);
    }
    else if(lonMode == 2) // lon - var2 - speed (m/s)
    {
        controlSpeed((double)var2, car);
    }
    else if(lonMode == 3) // lon - var2- accel-brake
    {
        setAcelerador((double)var2, car);
    }
    car->ctrl.gear = gearAuto(car);



    // Escribe los datos correspondientes
    sprintf(buffer, "%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f*", (float)angularError, (float)lateralError, (float)stretchType, (float)radius, (float)distToNextStretch, (float)nextStretchType, (float)nextRadius, (float)nextStretchLength, (float)currentSpeed);
    printf("%s\n", buffer);
    n = write(newsockfd, buffer, strlen(buffer));
    if (n < 0) error("ERROR writing to socket");



}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
    close(newsockfd);
    close(sockfd);
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
}

static void accelBrakeAuto(double currentTrack, double timeToNextStretch, double currentRadius, double nextRadius, tCarElt *car){

    if(currentTrack == 2)
        currentTrack = 1;
    if(currentTrack == 3)
        currentTrack = 0;

    /* Fuzzy STEERING Controller Inputs-------------------------------------------------- */
    std::vector<double> fuzzyInputs;
    fuzzyInputs.clear();
    fuzzyInputs.push_back(currentTrack);
    fuzzyInputs.push_back(timeToNextStretch);
    fuzzyInputs.push_back(currentRadius);
    fuzzyInputs.push_back(nextRadius);
    /* ---------------------------------------------------------------------------------- */

    /* Eval the fuzzy output -------------------------------------------------------------*/
    std::vector<double> fuzzyOutputs = mySpeedFuzzy->evalFis(fuzzyInputs);
    double targetSpeed = fuzzyOutputs.at(0);
    /* ---------------------------------------------------------------------------------- */

    controlSpeed(targetSpeed / 3.6, car);
}

static void steerAuto(double angularError, double lateralError, int segType, double radius, tCarElt *car){

    /* Fuzzy STEERING Controller Inputs-------------------------------------------------- */
    std::vector<double> fuzzyInputs;
    fuzzyInputs.clear();
    fuzzyInputs.push_back(angularError * 180 / M_PI);
    fuzzyInputs.push_back(lateralError);
    /* ---------------------------------------------------------------------------------- */

    /* Eval the fuzzy output -------------------------------------------------------------*/
    std::vector<double> fuzzyOutputs = mySteerFuzzy->evalFis(fuzzyInputs);
    double targetSteer = fuzzyOutputs.at(0);
    /* ---------------------------------------------------------------------------------- */

    /* Correcting steer with the predefined steer value in curve------------------------- */
    double forwardSteer = 0.0, offset = 0;
    double axelDist = car->priv.wheel[FRNT_LFT].relPos.x - car->priv.wheel[REAR_LFT].relPos.x;
    double steerLock = car->info.steerLock;

    if (segType == TR_RGT)
        offset = -lateralError;
    if(segType == TR_LFT)
        offset = lateralError;

    double alpha = atan2(axelDist, radius + offset);
    if(segType == TR_RGT)
        forwardSteer = -alpha / steerLock;
    if(segType == TR_LFT)
        forwardSteer = alpha / steerLock;
    /* ---------------------------------------------------------------------------------- */

    /* Actuate over the vehicle --------------------------------------------------------- */
    car->_steerCmd = forwardSteer + targetSteer;
    /* ---------------------------------------------------------------------------------- */
}

static void setSteerRadius(double radius, tCarElt* car){
    double axelDist = car->priv.wheel[FRNT_LFT].relPos.x - car->priv.wheel[REAR_LFT].relPos.x;
    double steerLock = car->info.steerLock;
    double alpha = atan2(axelDist, fabs(radius));
    double steer = alpha / steerLock;

    if(radius == 0)
        car->_steerCmd = 0;
    else if(radius < 0)
        car->_steerCmd = -steer;
    else
        car->_steerCmd = steer;
}

static int gearAuto(tCarElt *car) {

    int gear = car->_gear;

    const float SHIFT = 0.9; /* [-] (% of rpmredline) */
    const float SHIFT_MARGIN = 4.0; /* [m/s] */

    if (gear <= 0)
        gear = 1;

    float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
    float omega = car->_enginerpmRedLine / gr_up;
    float wr = car->_wheelRadius(2);

    if (omega * wr * SHIFT < car->_speed_x) {
        gear += 1;
    } else {
        float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
        omega = car->_enginerpmRedLine / gr_down;
        if (car->_gear > 1 && omega * wr * SHIFT > car->_speed_x + SHIFT_MARGIN) {
            gear -= 1;
        }
    }
    return gear;
}

static float getDistToNextStretch(tCarElt* car)
{
        tTrackSeg *seg = car->_trkPos.seg;
        float length = getDistToSegEnd(car);

        /* TR_RGT 1 Right curve
           TR_LFT 2 Left curve
           TR_STR 3 Straight */

        if (seg->type == TR_STR){
          while (seg->type == TR_STR) {
            seg = seg->next;
            length += seg->length;
          }
        }else{
          while (seg->type != TR_STR) {
            seg = seg->next;
            length += seg->length;
          }
        }

        return (length);
}


/* get the type of the next stretch */
static int getNextStretchType(tCarElt* car)
{
  tTrackSeg *seg = car->_trkPos.seg;

  /* TR_RGT 1 Right curve
     TR_LFT 2 Left curve
     TR_STR 3 Straight */

  if (seg->type == TR_STR)
  {
    while (seg->type == TR_STR) {
      seg = seg->next;
    }
  }
  else
  {
    while (seg->type != TR_STR) {
      seg = seg->next;
    }
  }
  return (seg->type);
}


/* get the radius of the next stretch */
static float getNextRadius(tCarElt* car)
{
  tTrackSeg *seg = car->_trkPos.seg;

  /* TR_RGT 1 Right curve
     TR_LFT 2 Left curve
     TR_STR 3 Straight */

  if (seg->type == TR_STR) {
    while (seg->type == TR_STR) {
      seg = seg->next;
    }
  } else {
    while (seg->type != TR_STR) {
      seg = seg->next;
    }
  }

  return (seg->radius);
}

static float getNextStretchLength(tCarElt* car){

    tTrackSeg *seg = car->_trkPos.seg;
    float length = 0;

    /* TR_RGT 1 Right curve
       TR_LFT 2 Left curve
       TR_STR 3 Straight */

    if (seg->type == TR_STR) {
      while (seg->type == TR_STR) {
        seg = seg->next;
      }
    } else {
      while (seg->type != TR_STR) {
        seg = seg->next;
      }
    }

    if(seg->type == TR_STR)
    {
        while (seg->type == TR_STR) {
            length += seg->length;
            seg = seg->next;
        }
    }
    else
    {
        while (seg->type != TR_STR) {
            length += seg->length;
            seg = seg->next;
        }
    }

    return length;
}



static float getDistToSegEnd(tCarElt* car)
{
  if (car->_trkPos.seg->type == TR_STR) {
    return car->_trkPos.seg->length - car->_trkPos.toStart;
  } else {
    return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
  }
}

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

static void controlSpeed(double targetSpeed, tCarElt* car)
{
  double speed = sqrt(car->_speed_x*car->_speed_x+car->_speed_y*car->_speed_y);
  double diff = speed-targetSpeed;
  double accbreak = 2.0 / (1.0+exp(diff)) - 1.0;

  if (accbreak == 0.0)
  {
      car->ctrl.accelCmd = 0;
      car->ctrl.brakeCmd = 0;
  }
  else if (accbreak < 0.0)
  {
      car->ctrl.accelCmd = 0;
      car->ctrl.brakeCmd = -accbreak;
  }
  else if (accbreak > 0.0)
  {
      car->ctrl.accelCmd = accbreak;
      car->ctrl.brakeCmd = 0;
  }
}

static void setAcelerador(double accBrake, tCarElt* car)
{
    if (accBrake == 0.0)
    {
        car->ctrl.accelCmd = 0;
        car->ctrl.brakeCmd = 0;
    }
    else if (accBrake < 0.0)
    {
        car->ctrl.accelCmd = 0;
        car->ctrl.brakeCmd = -accBrake;
    }
    else if (accBrake > 0.0)
    {
        car->ctrl.accelCmd = accBrake;
        car->ctrl.brakeCmd = 0;
    }
}
