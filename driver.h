/*
 * file: driver.h
 * created: Jun 15 2023
 * copyright: (C) 2023 Ashish Ahuja
 */

#ifndef _DRIVER_H_
#define _DRIVER_H_

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

#include "linalg.h"
#include "spline.h"

typedef struct trackNode {
	v2d v; // vector pointing at node location

	struct trackNode* next;
	struct trackNode* prev;

	tTrackSeg *seg;

	double radius;

	int flag; // 0 if nothing
#define TR_AP 1 // 1 if apex
} trackNode;

class Driver {
	public:
		Driver(int index);

		/* callback functions from TORCS */
		void initTrack(tTrack *t, void *carHandle, void **carParmHandle, tSituation *s);
		void newRace(tCarElt* car, tSituation *s);
		void drive(tCarElt *car, tSituation *s);
		int pitCommand(tCarElt *car, tSituation *s);
		void endRace(tCarElt *car, tSituation *s);

	private:
		void update(tCarElt *car, tSituation *s);
		float getAllowedSpeed(trackNode *node, float cur_speed, float mu);
		float getAccel(tCarElt* car);
		int getGear(tCarElt* car, int brake);
		float ABS(tCarElt *car, float brake);
		float getBrake(tCarElt *car);
		float getSteer(tCarElt *car);
		v2d getTargetPoint(tCarElt *car);
		float getDistToSegEnd(tCarElt *car);
		v2d getVectorAtSegDist(tTrackSeg *seg, float dist);
		trackNode *getTrackNodeAtDist(trackNode *node, int dist);
		v2d getNormalThroughTrackNode(trackNode *node);
		bool isPointInsideSegment(v2d point, tTrackSeg *seg, int full_seg);
		float getSlip(tCarElt *car);
		void initCa(tCarElt *car);
		void initCw(tCarElt *car);

		
		/* spline functions */
		void create_time_grid(std::vector<double>& T, double& tmin, double& tmax,
                      std::vector<double>& X, std::vector<double>& Y);
		trackNode* spline_racing_line(std::vector<v2d> points, int numNodes, tTrackSeg *seg);

		/* K199 implementation functions */
		double calcDeterminant(v2d a, v2d b);
		double getCurvature(trackNode *node);
		void racingLineIteration();

		float trackangle;
		float angle;

		static const float G;
		static const float FULL_ACCEL_MARGIN;
		static const float LOOKAHEAD_CONST;
		static const float LOOKAHEAD_FACTOR;
		static const float LOOKAHEAD_MAX;
		static const float MU_FACTOR;
		static const float TRACK_NODE_PARALLEL_DIST;
		static const float RACING_LINE_PERP_DIST;
		static const int TRACK_NODES_PER_SEG;
		static const int RACING_LINE_ITERATIONS;
		static const float SECURITY_MARGIN;
		static const float HALF_CAR_WIDTH;
		static const float CURB_WIDTH;
		static const float CAR_MASS;
		static const float MU_ALPHA;
		static const float MU_BETA;
		static const float PITBUILDING_SECURITY;
		static const float TC_SLIP;
		static const float TC_CUTOFF;
		static const float ABS_CUTOFF;
		static const float ABS_SLIP;

		int INDEX;
		
		tTrack *track;
		trackNode *start;
		trackNode *racingLine; // points to first track node of racing line
		int numTrackNodes; 
		int driveNumber;
		int launchControl;
		tk::spline torque_rpm_spline;
		float CA;
		float CW;
};

#endif

