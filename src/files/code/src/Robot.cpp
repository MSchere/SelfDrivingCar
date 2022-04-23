#include "Robot.h"

/* 
 * Module entry point  
 */
extern "C" int Robot0(tModInfo *modInfo) {
    initRobotIdentity(0, modInfo);
    return 0;
}

extern "C" int Robot1(tModInfo *modInfo) {
    initRobotIdentity(1, modInfo);
    return 0;
}

extern "C" int Robot2(tModInfo *modInfo) {
    initRobotIdentity(2, modInfo);
    return 0;
}

extern "C" int Robot3(tModInfo *modInfo) {
    initRobotIdentity(3, modInfo);
    return 0;
}

extern "C" int Robot4(tModInfo *modInfo) {
    initRobotIdentity(4, modInfo);
    return 0;
}

extern "C" int Robot5(tModInfo *modInfo) {
    initRobotIdentity(5, modInfo);
    return 0;
}

extern "C" int Robot6(tModInfo *modInfo) {
    initRobotIdentity(6, modInfo);
    return 0;
}

extern "C" int Robot7(tModInfo *modInfo) {
    initRobotIdentity(7, modInfo);
    return 0;
}

extern "C" int Robot8(tModInfo *modInfo) {
    initRobotIdentity(8, modInfo);
    return 0;
}

extern "C" int Robot9(tModInfo *modInfo) {
    initRobotIdentity(9, modInfo);
    return 0;
}

extern "C" int Robot10(tModInfo *modInfo) {
    initRobotIdentity(10, modInfo);
    return 0;
}

extern "C" int Robot11(tModInfo *modInfo) {
    initRobotIdentity(11, modInfo);
    return 0;
}

extern "C" int Robot12(tModInfo *modInfo) {
    initRobotIdentity(12, modInfo);
    return 0;
}

extern "C" int Robot13(tModInfo *modInfo) {
    initRobotIdentity(13, modInfo);
    return 0;
}

extern "C" int Robot14(tModInfo *modInfo) {
    initRobotIdentity(14, modInfo);
    return 0;
}

extern "C" int Robot15(tModInfo *modInfo) {
    initRobotIdentity(15, modInfo);
    return 0;
}

extern "C" int Robot16(tModInfo *modInfo) {
    initRobotIdentity(16, modInfo);
    return 0;
}

extern "C" int Robot17(tModInfo *modInfo) {
    initRobotIdentity(17, modInfo);
    return 0;
}

extern "C" int Robot18(tModInfo *modInfo) {
    initRobotIdentity(18, modInfo);
    return 0;
}

extern "C" int Robot19(tModInfo *modInfo) {
    initRobotIdentity(19, modInfo);
    return 0;
}

/* Module interface initialization. */
static int InitFuncPt(int index, void *pt) {
    tRobotItf *itf = (tRobotItf *) pt;

    /* for every track change or new race */
    itf->rbNewTrack = initTrack; /* Give the robot the track view called */
    itf->rbNewRace = newrace; /* Start a new race */
    itf->rbDrive = drive; /* Drive during race */
    itf->rbPitCmd = NULL;
    itf->rbEndRace = endrace; /* End of the current race */
    itf->rbShutdown = shutdown; /* Called before the module is unloaded */
    itf->index = index; /* Index used if multiple interfaces */
    return 0;
}

/* Called for every track change or new race. */
static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) {
    *carParmHandle = NULL;
}

/* Start a new race. */
static void newrace(int index, tCarElt* car, tSituation *s) {
    myDriver = new Driver(car, s);
}

/* Drive during race. */
static void drive(int index, tCarElt* car, tSituation *s) {
    memset((void *) &car->ctrl, 0, sizeof (tCarCtrl));
    myDriver->drive();
    double axelDist = car->priv.wheel[FRNT_LFT].relPos.x - car->priv.wheel[REAR_LFT].relPos.x;
            double steerLock = car->info.steerLock;
            printf("%lf %lf\n", axelDist, steerLock);
}

/* End of the current race */
static void endrace(int index, tCarElt *car, tSituation *s) {
}

/* Called before the module is unloaded */
static void shutdown(int index) {
}

void initRobotIdentity(int id, tModInfo *modInfo) {
    char robotName[20];
    sprintf(robotName, "Robot%d", id);

    memset(modInfo, 0, 10 * sizeof (tModInfo));
    modInfo->name = strdup(robotName); /* name of the module (short) */
    modInfo->desc = strdup(""); /* description of the module (can be long) */
    modInfo->fctInit = InitFuncPt; /* init function */
    modInfo->gfId = ROB_IDENT; /* supported framework version */
    modInfo->index = 1;
}
