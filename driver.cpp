/*
 * file: driver.cpp
 * created: Jun 15 2023
 * copyright: (C) 2023 Ashish Ahuja
 */

#include "driver.h"
#include <cmath>

const float Driver::G = 9.81;
const float Driver::FULL_ACCEL_MARGIN = 15.0; /* m/s */
const float Driver::LOOKAHEAD_CONST = 10;
//const float Driver::TRACK_NODE_PARALLEL_DIST = 30.0;
const float Driver::RACING_LINE_PERP_DIST = 0.1;
const int Driver::TRACK_NODES_PER_SEG = 4;
const int Driver::RACING_LINE_ITERATIONS = 100000;
const float Driver::HALF_CAR_WIDTH = 1.3;
const float Driver::CURB_WIDTH = 0.5;
const float Driver::CAR_MASS = 600; // kg
const float Driver::MU_ALPHA = 125;
const float Driver::MU_BETA = 0.6;
const float Driver::PITBUILDING_SECURITY = 0.2;

Driver::Driver(int index) {
	INDEX = index;
}

void Driver::initTrack(tTrack *t, void *carHandle, void **carParmHandle, tSituation *s) {
	driveNumber = 0;
	track = t;
	*carParmHandle = NULL;
	racingLine = NULL;

	tTrackSeg *seg = t->seg;
	trackNode *prev = NULL;
	numTrackNodes = 0;

	int idx = 0;

	do {
		float dist = seg->length / TRACK_NODES_PER_SEG;

		if ((idx++) % TRACK_NODES_PER_SEG == 0) { 
		for (int i = 0; i < 1; i ++) {
			v2d v = getVectorAtSegDist(seg, (float)i*dist);

			trackNode *node = (trackNode*)malloc(sizeof(trackNode));
			++numTrackNodes;
			node->next = NULL;
			node->prev = NULL;
			node->v = v;
			node->flag = 0;
			node->seg = seg;

			if (prev != NULL) {
				prev->next = node;
				node->prev = prev;
			} else {
				start = node;
			}

			prev = node;
		}}
		seg = seg->next;
	} while (seg != t->seg);

	start->prev = prev;
	prev->next = start;

	for (int i = 0; i < RACING_LINE_ITERATIONS; ++i) racingLineIteration();

	std::vector<v2d> racing_points;
	trackNode *node = start;
	do {
		racing_points.push_back(node->v);
		node = node->next;
	} while (node != start);
	
	start = spline_racing_line(racing_points, t->length, t->seg);

	/* debug print */
	trackNode *n = start;
	int tmp69 = 0;
	do {
		/*
		v2d nrm = getNormalThroughTrackNode(n);
		v2d a1 = n->v + 7*nrm;
		v2d a2 = n->v - 7*nrm;

		printf("%f, %f\n", a1.x, a1.y);
		printf("%f, %f\n", a2.x, a2.y);*/

		if (tmp69 % 4 == 0)
		printf("%f, %f\n", n->v.x, n->v.y);
		++tmp69;
		fflush(stdout);
		n = n->next;
	} while (n != start);

	racingLine = start;
}

bool Driver::isPointInsideSegment(v2d point, tTrackSeg *seg, int full_seg=0) {

	v2d v1, v2, v3, v4;
	Straight edges[4];

	v1 = v2d(seg->vertex[TR_SL].x, seg->vertex[TR_SL].y);
       	v2 = v2d(seg->vertex[TR_SR].x, seg->vertex[TR_SR].y);
       	v3 = v2d(seg->vertex[TR_EL].x, seg->vertex[TR_EL].y);
       	v4 = v2d(seg->vertex[TR_ER].x, seg->vertex[TR_ER].y);

       	edges[0] = Straight(v1, v2 - v1);
       	edges[1] = Straight(v1, v3 - v1);
       	edges[2] = Straight(v4, v3 - v4);
       	edges[3] = Straight(v4, v2 - v4);

	float left_curb = 0.0;
	float right_curb = 0.0;

	if (seg->lside->style == TR_CURB) left_curb = CURB_WIDTH;
	if (seg->rside->style == TR_CURB) right_curb = CURB_WIDTH;
	if (seg->barrier[TR_SIDE_LFT]->style == TR_PITBUILDING) left_curb = -PITBUILDING_SECURITY;
	if (seg->barrier[TR_SIDE_RGT]->style == TR_PITBUILDING) right_curb = -PITBUILDING_SECURITY;

	if (seg->type == TR_STR) {
		for (int i = 0; i < 2; ++i) {
			if (edges[i].opposite(point, v4) && edges[i].dist(point) > 0.01) return false;
		}

		for (int i = 2; i < 4; ++i) {
			if (edges[i].opposite(point, v1) && edges[i].dist(point) > 0.01) return false;
		}

		if (!full_seg)
		if (edges[1].dist(point) < (HALF_CAR_WIDTH - left_curb) || edges[3].dist(point) < (HALF_CAR_WIDTH - right_curb)) return false; // security boundary

		return true;
	}

	for (int i = 0; i < 1; ++i) {
		if (edges[i].opposite(point, v4) && edges[i].dist(point) > 0.01) return false;
	}

	for (int i = 2; i < 3; ++i) {
		if (edges[i].opposite(point, v1) && edges[i].dist(point) > 0.01) return false;
	}

	v2d c = v2d(seg->center.x, seg->center.y);

	float dist = c.dist(point);

	float r1;
	float r2;

	if (r1 > r2) {
		float tmp = r1;
		r1 = r2;
		r2 = tmp;
	}

	if (seg->type == TR_LFT) {
		r1 = seg->radiusl; if (!full_seg) r1 +=  HALF_CAR_WIDTH - left_curb;
		r2 = seg->radiusr; if (!full_seg) r2 += - HALF_CAR_WIDTH + right_curb;
	} else {
		r1 = seg->radiusr; if (!full_seg) r1+= + HALF_CAR_WIDTH - right_curb;
		r2 = seg->radiusl; if (!full_seg) r2 += - HALF_CAR_WIDTH + left_curb;
	}

	if (r1 <= dist && dist <= r2) return true;
	return false;
}

v2d Driver::getNormalThroughTrackNode(trackNode *node) {
	tTrackSeg *seg = node->seg;
	v2d ret;

	if (seg->type == TR_STR) ret = v2d(seg->rgtSideNormal.x, seg->rgtSideNormal.y);
	else
		ret = node->v - v2d(seg->center.x, seg->center.y);
	ret.normalize();
	return ret;
}

trackNode* Driver::getTrackNodeAtDist(trackNode *node, int dist) {
	if (dist == 0) return node;
	if (dist > 0) 
		return getTrackNodeAtDist(node->next, dist - 1);
	else 
		return getTrackNodeAtDist(node->prev, dist + 1);
}

double calcRadius(trackNode *node) {
	double x1 = node->v.x;
	double x2 = node->next->v.x;
	double x3 = node->prev->v.x;
	double y1 = node->v.y;
	double y2 = node->next->v.y;
	double y3 = node->prev->v.y;

	double A = x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2;
	double B = (x1*x1 + y1*y1)*(y3 - y2) + (x2*x2 + y2*y2) * (y1 - y3) + (x3*x3 +y3*y3)*(y2 - y1);
	double C = (x1*x1 + y1*y1)*(x2 - x3) + (x2*x2 + y2*y2)*(x3 - x1) + (x3*x3 +y3*y3)*(x1 - x2);
	double D = (x1*x1 + y1*y1)*(x3*y2 - x2*y3) +(x2*x2 +y2*y2)*(x1*y3 - x3*y1) + (x3*x3 + y3*y3)*(x2*y1 - x1*y2);

	if (A == 0) return FLT_MAX;
	return sqrt((B*B + C*C - 4*A*D)/(4*A*A));
}

void Driver::newRace(tCarElt *car, tSituation *s) {
}

void Driver::drive(tCarElt *car, tSituation *s) {
	update(car, s);

	memset(&car->ctrl, 0, sizeof(tCarCtrl));

	// call steer after brake!! cause steer modifies racingLine variable
	car->ctrl.brakeCmd = getBrake(car);
	car->ctrl.steer = getSteer(car);

	if (car->ctrl.brakeCmd == 0.0) {
		car->ctrl.accelCmd = getAccel(car);
		car->ctrl.gear = getGear(car, 0);
	} else {
		car->ctrl.accelCmd = 0.0;
		car->ctrl.gear = getGear(car, 1);
	}

}

int Driver::pitCommand(tCarElt *car, tSituation *s) {
	return ROB_PIT_IM;
}

void Driver::endRace(tCarElt *car, tSituation *s) {
}

void Driver::update(tCarElt *car, tSituation *s) {
	v2d v_parallel = racingLine->v - racingLine->prev->v;

	v_parallel = v_parallel.rotate(v2d(0, 0), 3.14159/2);
	Straight str = Straight(racingLine->v, v_parallel);

	while (str.opposite(v2d(car->_pos_X, car->_pos_Y), racingLine->prev->v)) {
		racingLine = racingLine->next;
		v_parallel = racingLine->v - racingLine->prev->v;
		v_parallel = v_parallel.rotate(v2d(0, 0), 3.14159/2);

		str = Straight(racingLine->v, v_parallel);
	}

	trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
	angle = trackangle - car->_yaw;
	NORM_PI_PI(angle);
}

float Driver::getAllowedSpeed(trackNode *node, float cur_speed, float mu) {
	double curv = getCurvature(node); 

	if (curv < 0) curv *= -1;

	if (curv == 0) return FLT_MAX;


	float radius = 1 / curv;

	float ret = sqrt((mu*mu*MU_ALPHA*MU_ALPHA*radius*radius) + 
			(4 * mu * CAR_MASS * radius * G * (CAR_MASS - (mu * MU_BETA * radius))));
	ret += (mu * MU_ALPHA * radius);
	ret /= 2;
	ret /= (CAR_MASS - (mu * MU_BETA * radius));


	if (ret < 0) return FLT_MAX;

	if (ret != ret) return FLT_MAX;
	//assert(ret == ret); // checking for nan

	/*
	float ret = mu * CAR_MASS * G * radius;
	ret /= (CAR_MASS - mu * MU_BETA * radius);
	ret = sqrt(ret);

	if (ret != ret) {
		puts("nan eh");
		return FLT_MAX;
	}*/

	/*
	float ret = (mu*mu*MU_ALPHA*MU_ALPHA*radius*radius) + (4*mu*CAR_MASS*CAR_MASS*G*radius);
	ret = sqrt(ret) + (mu*MU_ALPHA*radius);
	ret /= (2 * CAR_MASS);

	if (ret != ret) {
		return FLT_MAX;
	}*/

	return ret;
}

float Driver::getDistToSegEnd(tCarElt *car) {
	if (car->_trkPos.seg->type == TR_STR) {
		return car->_trkPos.seg->length - car->_trkPos.toStart;
	}
	return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
}

float Driver::getAccel(tCarElt *car) {
	float allowedspeed = getAllowedSpeed(racingLine, car->_speed_x, car->_trkPos.seg->surface->kFriction);
	float gr = car->_gearRatio[car->_gear + car->_gearOffset];
	float rm = car->_enginerpmRedLine;
	if (allowedspeed > car->_speed_x + FULL_ACCEL_MARGIN) return 1.0;
	return allowedspeed/car->_wheelRadius(REAR_RGT)*gr/rm;
}

int Driver::getGear(tCarElt *car, int brake) {
	float change_up = car->_enginerpmRedLine * 0.99;
	float change_down = car->_enginerpmRedLine * 0.75;
	float rpm = car->_enginerpm;
	float cur_gear = car->_gear; 

	if (rpm > change_up && cur_gear != 7) {
		return cur_gear + 1;
	} else if (rpm < change_down && cur_gear != 1 && brake) {
		return cur_gear - 1;
	}
	return cur_gear;
}

float Driver::getBrake(tCarElt *car) {
	tTrackSeg *segptr = car->_trkPos.seg;
	trackNode *rptr = racingLine;
	float currentspeedsqr = car->_speed_x*car->_speed_x;
	float mu = segptr->surface->kFriction;
	float maxlookahead = currentspeedsqr / (2.0 * mu * G);

	v2d cur_pos = v2d(car->_pos_X, car->_pos_Y);
	float lookahead = racingLine->v.dist(cur_pos);

	float allowedspeed = getAllowedSpeed(racingLine, car->_speed_x, mu);
	if (allowedspeed < car->_speed_x) {
		float delta = car->_speed_x - allowedspeed;
		if (delta > 10) return 1.0;
		return delta / 10;
	}


	segptr = segptr->next;
	while (lookahead < maxlookahead) {
		allowedspeed = getAllowedSpeed(rptr, car->_speed_x, mu);
		if (allowedspeed < car->_speed_x) {
			float cs = car->_speed_x;
			float as = allowedspeed;

			float brakedist = log(MU_BETA * cs * cs + MU_ALPHA * cs + CAR_MASS * G) -
				log(MU_BETA * as * as + MU_ALPHA * as + CAR_MASS * G);

			float tmp3 = sqrt(abs(4 * MU_BETA * CAR_MASS * G - MU_ALPHA * MU_ALPHA));
			assert(tmp3 == tmp3);
			assert(brakedist == brakedist);

			brakedist += (2 * MU_ALPHA * (atan((MU_ALPHA + 2 * MU_BETA * as)/tmp3) - 
					atan((MU_ALPHA + 2 * MU_BETA * cs)/tmp3)))/tmp3;

			brakedist *= CAR_MASS / (2 * MU_BETA * mu);

			assert(brakedist == brakedist);
			if (brakedist < 0) brakedist *= -1;
			assert(brakedist >= 0);


					/*
			float brakedist = log(MU_ALPHA * cs + CAR_MASS * G) - 
				log(MU_ALPHA * as + CAR_MASS * G);
			brakedist *= (CAR_MASS * G / MU_ALPHA);
			brakedist += (as - cs);

			brakedist *= (CAR_MASS)/(MU_ALPHA * mu) * (-1);

			assert(brakedist == brakedist);
			assert(brakedist >= 0);*/

			/*
			float brakedist = log(MU_BETA * cs * cs + CAR_MASS * G) -
				log(MU_BETA * as * as + CAR_MASS * G);
			brakedist *= (CAR_MASS)/(2 * mu * MU_BETA);*/

			/*

			double tmp1 = log(mu * MU_BETA * as * as + mu * MU_ALPHA * as + mu * CAR_MASS * G) - 
					log(mu * MU_BETA * cs * cs + mu * MU_ALPHA * cs + mu * CAR_MASS * G);
			tmp1 *= CAR_MASS / (2 * mu * MU_BETA);

			double tmp2 = sqrt(4 * mu * MU_BETA * mu * CAR_MASS * G - mu * mu * MU_ALPHA * MU_ALPHA);
			
			double tmp3 = atan((2 * mu * MU_BETA * cs + mu * MU_ALPHA)/tmp2) - 
					atan((2 * mu * MU_BETA * as + mu * MU_ALPHA)/tmp2);
			tmp3 *= CAR_MASS * mu * MU_ALPHA / (mu * MU_BETA * tmp2);

			float brakedist = tmp1 + tmp3;

			assert(brakedist >= 0);
			assert(brakedist == brakedist);*/

			if (brakedist > lookahead) {
				float delta = car->_speed_x - allowedspeed;
				if (delta > 10) return 1.0;
				return delta / 10;
			}
		}
		rptr = rptr->next;
		lookahead = rptr->v.dist(cur_pos); 
	}
	return 0.0;
}

v2d Driver::getTargetPoint(tCarElt *car) {
	trackNode *node = racingLine;

	for (int i = 0; i < LOOKAHEAD_CONST; ++i) node = node->next;
	return node->v;

	/*
    	float lookahead = LOOKAHEAD_CONST + car->_speed_x*LOOKAHEAD_FACTOR;

	float length = getDistToSegEnd(car); 

	while (length < lookahead) {
		seg = seg->next;
		length += seg->length;
	}

	length = lookahead - length + seg->length;
	v2d s;
	s.x = (seg->vertex[TR_SL].x + seg->vertex[TR_SR].x) / 2;
	s.y = (seg->vertex[TR_SL].y + seg->vertex[TR_SR].y) / 2;

	if (seg->type == TR_STR) {
		tTrackSeg *segcp = seg;
		float dist_covered = 0;
		while (seg->type == TR_STR && dist_covered < LOOKAHEAD_MAX) {
			dist_covered += seg->length;
			seg = seg->next;
		}

		if (seg->type == TR_STR) {
			seg = segcp;
			v2d d;
			d.x = (seg->vertex[TR_EL].x - seg->vertex[TR_SL].x) / seg->length;
			d.y = (seg->vertex[TR_EL].y - seg->vertex[TR_SL].y) / seg->length;
			return s + d*length;
		}

		float leftratio = 0.95, rightratio = 0.05;
		if (seg->type == TR_LFT) {
			leftratio = 0.05;
			rightratio = 0.95;
		}
		v2d d = v2d(leftratio * seg->vertex[TR_SL].x + rightratio * seg->vertex[TR_SR].x,
				leftratio * seg->vertex[TR_SL].y + rightratio * seg->vertex[TR_SR].y);
		v2d cpos = v2d(car->_pos_X, car->_pos_Y);
		Straight s1 = Straight(cpos, d - cpos);

		seg = segcp;
		v2d s2d = v2d(seg->vertex[TR_SL].x - seg->vertex[TR_SR].x,
				seg->vertex[TR_SL].y - seg->vertex[TR_SR].y);
		v2d anchor = v2d(seg->vertex[TR_SL].x, seg->vertex[TR_SL].y);
		Straight s2 = Straight(anchor, s2d);

		return s1.intersect(s2); 
	} else {
		return getVectorAtSegDist(seg, length);
	}*/
}

v2d Driver::getVectorAtSegDist(tTrackSeg *seg, float dist) {
	v2d s;
	s.x = (seg->vertex[TR_SL].x + seg->vertex[TR_SR].x) / 2;
	s.y = (seg->vertex[TR_SL].y + seg->vertex[TR_SR].y) / 2;

	if (seg->type == TR_STR) { 
		v2d d;
		d.x = (seg->vertex[TR_EL].x - seg->vertex[TR_SL].x) / seg->length;
		d.y = (seg->vertex[TR_EL].y - seg->vertex[TR_SL].y) / seg->length;
		return s + d*dist;
	} else {
		v2d c;
		c.x = seg->center.x;
		c.y = seg->center.y;
		float arc = dist/seg->radius;
		float arcsign = (seg->type == TR_RGT) ? -1 : 1;
		arc *= arcsign;
		return s.rotate(c, arc);
	}
}

float Driver::getSteer(tCarElt *car) {
	float targetAngle;
	v2d target = getTargetPoint(car);

	targetAngle = atan2(target.y - car->_pos_Y, target.x - car->_pos_X);
	targetAngle -= car->_yaw;
	NORM_PI_PI(targetAngle);
	return targetAngle / car->_steerLock;
}

/* K199 implementation */
/* Ref: Remi Coulom's PhD Thesis Appendix C (https://www.remi-coulom.fr/Publications/Thesis.pdf) */

double Driver::calcDeterminant(v2d a, v2d b) {
	return (a.x * b.y) - (a.y * b.x);
}

double Driver::getCurvature(trackNode *node) {
	v2d x1 = node->prev->v;
	v2d x2 = node->v;
	v2d x3 = node->next->v;

	assert(x1.x != x2.x || x1.y != x2.y);
	assert(x1.x != x3.x || x1.y != x3.y);
	assert(x3.x != x2.x || x3.y != x2.y);

	double ret = 2.0;
	ret *= calcDeterminant(x3 - x2, x1 - x2);

	v2d d1 = x2 - x1; 
        v2d d2 = x3 - x1;	
	v2d d3 = x3 - x2;

	double a1 = d1.len();
	double a2 = d2.len();
	double a3 = d3.len();

	ret /= (a1 * a2 * a3);

	return ret;
}

void Driver::racingLineIteration() {
       trackNode *node = start;	

       v2d new_vectors[numTrackNodes];
       int idx = 0;

       do {
	       v2d v = node->v;
	       assert(isPointInsideSegment(v, node->seg));
	
	       double c1 = getCurvature(node->prev);
	       double c2 = getCurvature(node->next); 

	       double delta = FLT_MAX;
	       v2d new_v = v2d(-1, -1);

	       v2d nrm = getNormalThroughTrackNode(node);

	       double dmax = node->seg->width;

	       if (dmax > 0.5) dmax = 0.5;
		
	       assert(dmax > 0);

	       for (double dist = 0.0; dist <= dmax; dist += RACING_LINE_PERP_DIST) {
		      v2d tmp_v = v + (dist * nrm); 

		      if (!isPointInsideSegment(tmp_v, node->seg)) break;

		      double dist_1 = tmp_v.dist(node->prev->v); 
		      double dist_2 = tmp_v.dist(node->next->v);

		      assert(dist_1 != 0);
		      assert(dist_2 != 0);

		      double ideal_curvature = (dist_1 * c1 + dist_2 * c2) / (dist_1 + dist_2);
		      node->v = tmp_v;
		      double real_curvature = getCurvature(node);
		      node->v = v;

		      double tmp_delta = abs(real_curvature - ideal_curvature);


		      if (tmp_delta < delta) {
			      delta = tmp_delta;
			      new_v = tmp_v;
		      }
	       }

	       for (double dist = 0.0; dist >= -dmax; dist -= RACING_LINE_PERP_DIST) {
		      v2d tmp_v = v + (dist * nrm); 

		      if (!isPointInsideSegment(tmp_v, node->seg)) break;

		      double dist_1 = tmp_v.dist(node->prev->v); 
		      double dist_2 = tmp_v.dist(node->next->v);

		      assert(dist_1 != 0);
		      assert(dist_2 != 0);

		      double ideal_curvature = (dist_1 * c1 + dist_2 * c2) / (dist_1 + dist_2);
		      node->v = tmp_v;
		      double real_curvature = getCurvature(node);
		      node->v = v;

		      double tmp_delta = abs(real_curvature - ideal_curvature);

		      if (tmp_delta < delta) {
			      delta = tmp_delta;
			      new_v = tmp_v;
		      }
	       }

	       assert(new_v.x != -1);

	       node->v = new_v;
	       node = node->next;

       } while (node != start);
}

trackNode* Driver::spline_racing_line(std::vector<v2d> points, int numNodes, tTrackSeg *seg) {
	int n = points.size();

	std::vector<double> X, Y, T;
	for (int i = 0; i < n; ++i) {
		X.push_back(points[i].x);
		Y.push_back(points[i].y);
	}

	T.resize(X.size());
	double tmin = 0.0;
	double tmax = 0.0;

	create_time_grid(T,tmin,tmax,X,Y);

	tk::spline::spline_type type = tk::spline::cspline;
	tk::spline sx, sy;
    	sx.set_points(T,X,type);
    	sy.set_points(T,Y,type);

	trackNode *ret = NULL;
	trackNode *prev = NULL;

 	for (int i = 0; i < numNodes - 1; ++i) {
		double t = tmin + (double)i*(tmax - tmin)/(numNodes - 1);

		v2d v = v2d(sx(t), sy(t));

		trackNode *node = (trackNode*)malloc(sizeof(trackNode));
		node->next = NULL;
		node->prev = NULL;
		node->flag = 0;
		node->radius = 0;
		node->seg = NULL;

		node->v = v;
		if (prev != NULL) {
			prev->next = node;
			node->prev = prev;
		} else ret = node;

		prev = node;
	}

	ret->prev = prev;
	prev->next = ret;

	return ret;

}


void Driver::create_time_grid(std::vector<double>& T, double& tmin, double& tmax,
                      std::vector<double>& X, std::vector<double>& Y)
{

    assert(X.size()==Y.size() && X.size()>2);

    // hack for closed curves (so that it closes smoothly):
    //  - append the same grid points a few times so that the spline
    //    effectively runs through the closed curve a few times
    //  - then we only use the last loop
    //  - if periodic boundary conditions were implemented then
    //    simply setting periodic bd conditions for both x and y
    //    splines is sufficient and this hack would not be needed
    int idx_first=-1, idx_last=-1;
    if(69 > 1) {
        // remove last point if it is identical to the first
        if(X[0]==X.back() && Y[0]==Y.back()) {
            X.pop_back();
            Y.pop_back();
        }

        const int num_loops=3;  // number of times we go through the closed loop
        std::vector<double> Xcopy, Ycopy;
        for(int i=0; i<num_loops; i++) {
            Xcopy.insert(Xcopy.end(), X.begin(), X.end());
            Ycopy.insert(Ycopy.end(), Y.begin(), Y.end());
        }
        idx_last  = (int)Xcopy.size()-1;
        idx_first = idx_last - (int)X.size();
        X = Xcopy;
        Y = Ycopy;

        // add first point to the end (so that the curve closes)
        X.push_back(X[0]);
        Y.push_back(Y[0]);
    }

    // setup a "time variable" so that we can interpolate x and y
    // coordinates as a function of time: (X(t), Y(t))
    T.resize(X.size());
    T[0]=0.0;
    for(size_t i=1; i<T.size(); i++) {
        // time is proportional to the distance, i.e. we go at a const speed
        T[i] = T[i-1] + sqrt( pow(X[i]-X[i-1], 2) + pow(Y[i]-Y[i-1], 2) );
    }
    if(idx_first<0 || idx_last<0) {
        tmin = T[0] - 0.0;
        tmax = T.back() + 0.0;
    } else {
        tmin = T[idx_first];
        tmax = T[idx_last];
    }
}

