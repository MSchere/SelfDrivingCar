/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ContextInfo.cpp
 * Author: ruben
 * 
 * Created on 6 de octubre de 2016, 11:02
 */

#include "ContextInfo.h"
//#include "CpuTime.h"

ContextInfo::ContextInfo(tCarElt* car, tSituation *s) {
    m_car = car;
    m_s = s;
    m_currentTime = -1e+5;
    m_acuCurrentTime = 0;
    this->updateContext();
}

ContextInfo::~ContextInfo() {
}

void ContextInfo::updateContext() {

    if (m_car->race.curLapTime < (m_currentTime - m_acuCurrentTime))
        m_acuCurrentTime = m_currentTime;
    m_currentTime = m_car->race.curLapTime + m_acuCurrentTime;

    m_allCarsContext.clear();
    for (int i = 0; i < m_s->raceInfo.ncars; i++)
        m_allCarsContext.push_back(this->computeContext(m_s->cars[i]));
    m_context = this->computeContext(m_car);
    m_relativePositions = this->carsPosition(m_car, m_s, m_allCarsContext);
    m_surround = this->computeSurround(m_car, m_s);
}

contextInfo_t ContextInfo::getMyContextInfo() {
    return m_context;
}

contextInfo_t ContextInfo::getCarContextInfo(int id) {
    if (id >= 0 && id < m_allCarsContext.size())
        return m_allCarsContext.at(id);
    else {
        printf("CantexInfo::getCarContextInfo(int index): error, index is outside of contextInfo vector\n");
        exit - 1;
    }
}

std::vector<contextInfo_t> ContextInfo::getAllCarsContextInfo() {
    return m_allCarsContext;
}

std::vector<relativePosition_t> ContextInfo::getRelativePositions() {
    return m_relativePositions;
}

surroundingVehicles_t ContextInfo::getSurround() {
    return m_surround;
}

double ContextInfo::getCurrentTime() {
    return m_currentTime;
}

double ContextInfo::getCenterForLaneId(int laneId) {
    double centerLane = this->computeLaneCenter(m_context.road, laneId);
    return centerLane;
}

roadStruct_t ContextInfo::computeRoadStruct(tCarElt* car, double minLaneWidth) {
    roadStruct_t road;
    road.width = car->pub.trkPos.seg->width;
    road.numberLanes = floor(road.width / minLaneWidth);
    road.laneWidth = road.width / road.numberLanes;
    road.laneCenter.clear();
    for (int i = 0; i < road.numberLanes; i++){
        road.laneCenter.push_back(this->computeLaneCenter(road, i));
        road.laneStartFromRightLimit.push_back(i * road.laneWidth);
        road.laneCenterFromRightLimit.push_back(i * road.laneWidth + road.laneWidth / 2.0);
        road.laneEndFromRightLimit.push_back((i + 1) * road.laneWidth);
    }

    return road;
}

double ContextInfo::computeLaneCenter(roadStruct_t road, int laneId) {

    double laneCenter = 0;

    if (laneId >= 0 || laneId < road.numberLanes)
        laneCenter = -road.width / 2.0 + road.laneWidth * (0.5 + double(laneId));
    else
        printf("Error: ContextInfo::getCenterForLane(int index), index >= 0 & index <= m_context.numberLanes\n");

    return laneCenter;
}

contextInfo_t ContextInfo::computeContext(tCarElt* car) {

    contextInfo_t context;
    context.dynamics = this->computeDynamics(car);
    context.absoluteCorners = this->computeCarCorners(car, CTX_CAR_CORNERS_ABS);
    context.relativeCorners = this->computeCarCorners(car, CTX_CAR_CORNERS_REL);
    context.road = this->computeRoadStruct(car, CTX_LANE_WIDTH);
    context.dist2RoadCenter = car->pub.trkPos.toMiddle;
    context.dist2RightLimit = context.dist2RoadCenter + context.road.width / 2.0;
    context.distRaced = car->race.distRaced;
    context.mainLaneId = this->computeMainLaneId(context.road, context.dist2RoadCenter);
    context.dist2MainLaneCenter = this->computeDist2Lane(context.road, context.mainLaneId, context.dist2RoadCenter);
    context.secondLaneId = this->computeSecondLaneId(context.road, context.relativeCorners, context.mainLaneId);
    context.dist2SecondLaneCenter = this->computeDist2Lane(context.road, context.secondLaneId, context.dist2RoadCenter);

    return context;
}

dynamics_t ContextInfo::computeDynamics(tCarElt* car) {

    dynamics_t dynamics;
    dynamics.posX = car->pub.DynGC.pos.x;
    dynamics.posY = car->pub.DynGC.pos.y;
    dynamics.speedX = car->pub.DynGC.vel.x;
    dynamics.speedY = car->pub.DynGC.vel.y;
    dynamics.accelX = car->pub.DynGC.acc.x;
    dynamics.accelY = car->pub.DynGC.acc.y;
    dynamics.heading = car->pub.DynGC.pos.az;
    dynamics.yawRate = car->pub.DynGC.vel.az;
    dynamics.dist2RoadCenter = car->pub.trkPos.toMiddle;
    dynamics.headingError = RtTrackSideTgAngleL(&(m_car->pub.trkPos)) - m_car->pub.DynGC.pos.az;

    return dynamics;
}

surroundingVehicles_t ContextInfo::computeSurround(tCarElt* car, tSituation *s) {

    posVehicle_t defaultVehicle{false, 0, 0, 0};
    sideLane_t defatultSideLane{false, 0, defaultVehicle, defaultVehicle, defaultVehicle};
    surroundingVehicles_t sv{defatultSideLane, defatultSideLane, defatultSideLane};

    for (int laneId = 0; laneId < m_context.road.numberLanes; laneId++) {
        if (laneId == m_context.mainLaneId + 1) {
            sv.left.lane = true;
            sv.left.laneId = laneId;
            sv.left.front = this->getFrontCar(car, laneId, m_relativePositions);
            sv.left.middle = this->getMiddleCar(car, laneId, m_relativePositions);
            sv.left.rear = this->getRearCar(car, laneId, m_relativePositions);
        }
        if (laneId == m_context.mainLaneId) {
            sv.center.lane = true;
            sv.center.laneId = laneId;
            sv.center.front = this->getFrontCar(car, laneId, m_relativePositions);
            sv.center.middle = this->getMiddleCar(car, laneId, m_relativePositions);
            sv.center.rear = this->getRearCar(car, laneId, m_relativePositions);
        }
        if (laneId == m_context.mainLaneId - 1) {
            sv.right.lane = true;
            sv.right.laneId = laneId;
            sv.right.front = this->getFrontCar(car, laneId, m_relativePositions);
            sv.right.middle = this->getMiddleCar(car, laneId, m_relativePositions);
            sv.right.rear = this->getRearCar(car, laneId, m_relativePositions);
        }
    }

    return sv;
}

posVehicle_t ContextInfo::getFrontCar(tCarElt* car, int evalLaneID, std::vector<relativePosition_t> positions) {

    posVehicle_t vehicle{false, true, 1e+10, 0};
    for (int i = 0; i < positions.size(); i++)
        if (evalLaneID == positions.at(i).mainLaneId || evalLaneID == positions.at(i).secondLaneId)
            if (car->index != positions.at(i).id)
                if (positions.at(i).usedDist > 0.0)
                    if (positions.at(i).usedDist < vehicle.dist) {
                        vehicle.car = true;
                        vehicle.id = positions.at(i).id;
                        vehicle.dist = positions.at(i).usedDist;
                        vehicle.speed = m_allCarsContext.at(i).dynamics.speedX;
                    }

    return vehicle;
}

posVehicle_t ContextInfo::getMiddleCar(tCarElt* car, int evalLaneID, std::vector<relativePosition_t> positions) {

    posVehicle_t vehicle{false, true, car->info.dimension.x, 0};
    for (int i = 0; i < positions.size(); i++)
        if (evalLaneID == positions.at(i).mainLaneId || evalLaneID == positions.at(i).secondLaneId)
            if (car->index != positions.at(i).id)
                if (positions.at(i).usedDist == 0.0)
                    if (fabs(positions.at(i).usedDist) < fabs(vehicle.dist)) {
                        vehicle.car = true;
                        vehicle.id = positions.at(i).id;
                        vehicle.dist = positions.at(i).usedDist;
                        vehicle.speed = m_allCarsContext.at(i).dynamics.speedX;
                    }

    return vehicle;
}

posVehicle_t ContextInfo::getRearCar(tCarElt* car, int evalLaneID, std::vector<relativePosition_t> positions) {

    posVehicle_t vehicle{false, true, -1e+10, 0};
    for (int i = 0; i < positions.size(); i++)
        if (evalLaneID == positions.at(i).mainLaneId || evalLaneID == positions.at(i).secondLaneId)
            if (car->index != positions.at(i).id)
                if (positions.at(i).usedDist < 0.0)
                    if (positions.at(i).usedDist > vehicle.dist) {
                        vehicle.car = true;
                        vehicle.id = positions.at(i).id;
                        vehicle.dist = positions.at(i).usedDist;
                        vehicle.speed = m_allCarsContext.at(i).dynamics.speedX;
                    }

    return vehicle;
}

carCorners_t ContextInfo::computeCarCorners(tCarElt* car, int mode) {

    double semiLength = car->info.dimension.x / 2.0;
    double semiWidth = car->info.dimension.y / 2.0;
    double segment = hypot(semiLength, semiWidth);

    double referenceHeading, centerX, centerY;
    if (mode == CTX_CAR_CORNERS_ABS) {
        referenceHeading = car->pub.DynGC.pos.az;
        centerX = car->pub.DynGC.pos.x;
        centerY = car->pub.DynGC.pos.y;
    } else if (mode == CTX_CAR_CORNERS_REL) {
        referenceHeading = -(RtTrackSideTgAngleL(&(car->pub.trkPos)) - car->pub.DynGC.pos.az);
        centerX = 0;
        centerY = car->pub.trkPos.toMiddle;
    } else
        printf("Error: ContextInfo::computeCarCorners(), mode not valid\n");

    double FLAngle = referenceHeading + atan2(+semiWidth, +semiLength);
    double FRAngle = referenceHeading + atan2(-semiWidth, +semiLength);
    double RLAngle = referenceHeading + atan2(+semiWidth, -semiLength);
    double RRAngle = referenceHeading + atan2(-semiWidth, -semiLength);

    carCorners_t corners;
    corners.FL.x = centerX + cos(FLAngle) * segment;
    corners.FL.y = centerY + sin(FLAngle) * segment;
    corners.FR.x = centerX + cos(FRAngle) * segment;
    corners.FR.y = centerY + sin(FRAngle) * segment;
    corners.RL.x = centerX + cos(RLAngle) * segment;
    corners.RL.y = centerY + sin(RLAngle) * segment;
    corners.RR.x = centerX + cos(RRAngle) * segment;
    corners.RR.y = centerY + sin(RRAngle) * segment;

    return corners;
}

int ContextInfo::computeMainLaneId(roadStruct_t road, double dist2RoadCenter) {
    int laneId = 0;
    double lessDist2LaneCenter = 1e+5;

    for (int i = 0; i < road.numberLanes; i++) {
        double dist2LaneCenter = dist2RoadCenter - road.laneCenter.at(i);// this->computeDist2Lane(road, i, dist2RoadCenter);
        if (fabs(dist2LaneCenter) < fabs(lessDist2LaneCenter)) {
            lessDist2LaneCenter = dist2LaneCenter;
            laneId = i;
        }
    }
    return laneId;
}

int ContextInfo::computeSecondLaneId(roadStruct_t road, carCorners_t relativeCorners, int mainLaneId) {
    int secondLaneId = mainLaneId;
    int leftLane = mainLaneId + 1;
    int rightLane = mainLaneId - 1;

    bool testR = true;
    if (rightLane < 0)
        testR = false;
    if (testR) {
        if (this->computeMainLaneId(road, relativeCorners.FL.y) == rightLane)
            secondLaneId = rightLane;
        if (this->computeMainLaneId(road, relativeCorners.FR.y) == rightLane)
            secondLaneId = rightLane;
        if (this->computeMainLaneId(road, relativeCorners.RL.y) == rightLane)
            secondLaneId = rightLane;
        if (this->computeMainLaneId(road, relativeCorners.RR.y) == rightLane)
            secondLaneId = rightLane;
    }

    bool testL = true;
    if (leftLane >= road.numberLanes)
        testL = false;
    if (testL) {
        if (this->computeMainLaneId(road, relativeCorners.FL.y) == leftLane)
            secondLaneId = leftLane;
        if (this->computeMainLaneId(road, relativeCorners.FR.y) == leftLane)
            secondLaneId = leftLane;
        if (this->computeMainLaneId(road, relativeCorners.RL.y) == leftLane)
            secondLaneId = leftLane;

        if (this->computeMainLaneId(road, relativeCorners.RR.y) == leftLane)
            secondLaneId = leftLane;
    }

    return secondLaneId;
}

double ContextInfo::computeDist2Lane(roadStruct_t road, int laneId, double dist2RoadCenter) {
    double laneCenter = this->computeLaneCenter(road, laneId);
    double dist2LaneCenter = dist2RoadCenter - laneCenter;

    return dist2LaneCenter;
}

std::vector<relativePosition_t> ContextInfo::carsPosition(tCarElt* car, tSituation *s, std::vector<contextInfo_t> context) {

    std::vector<relativePosition_t> positions;
    double myCarSemiLength = car->info.dimension.x / 2.0;

    for (int i = 0; i < context.size(); i++) {
        double otherCarSemiLength = s->cars[i]->info.dimension.x / 2.0;
        relativePosition_t position;
        position.id = s->cars[i]->index;
        position.mainLaneId = context.at(i).mainLaneId;
        position.secondLaneId = context.at(i).secondLaneId;
        int otherTrackId = s->cars[i]->pub.trkPos.seg->id;

        tTrackSeg *frontSeg = car->pub.trkPos.seg;
        position.frontDist = s->cars[i]->pub.trkPos.toStart - otherCarSemiLength;
        position.frontDist -= car->pub.trkPos.toStart + myCarSemiLength;
        while (frontSeg->id != otherTrackId) {
            frontSeg = frontSeg->next;
            position.frontDist += frontSeg->length;
        }
        tTrackSeg *rearSeg = car->pub.trkPos.seg;
        position.rearDist = s->cars[i]->pub.trkPos.toStart + otherCarSemiLength;
        position.rearDist -= car->pub.trkPos.toStart - myCarSemiLength;
        while (rearSeg->id != otherTrackId) {
            rearSeg = rearSeg->prev;
            position.rearDist -= frontSeg->length;
        }

        // Comprobar que el coche no esta a nuestro lado
        if (position.frontDist < 0.0)
            position.frontDist = 0.0;
        if (position.rearDist > 0.0)
            position.rearDist = 0.0;
        // Nos quedamos con la menor de las distancias
        if (position.frontDist <= -position.rearDist)
            position.usedDist = position.frontDist;
        else
            position.usedDist = position.rearDist;

        positions.push_back(position);
    }

    return positions;
}