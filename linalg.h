/*
 * file: linalg.h
 * created: Jun 16 2023
 */

#ifndef _LINALG_H_
#define _LINALG_H_

class v2d{
	public:
		v2d() {}
		v2d(const v2d &src) { this->x = src.x; this->y = src.y; }
		v2d(float x, float y) { this->x = x; this->y = y; }

		v2d& operator=(const v2d &src);
		v2d operator+(const v2d &src) const;
		v2d operator-(void) const;
		v2d operator-(const v2d &src) const;
		v2d operator*(const float s) const;
		float operator*(const v2d &src) const;
		friend v2d operator*(const float s, const v2d &src); 

		float len(void) const;
		void normalize(void);
		float dist(const v2d &p) const;
		float cosalpha(const v2d &p2, const v2d &center) const;
		v2d rotate(const v2d &c, float arc) const;

		float x, y;
};

class Straight{
	public:
		Straight() {}
		Straight(float x, float y, float dx, float dy) {
			p.x = x; p.y = y; d.x = dx; d.y = dy;
			d.normalize();
		}
		Straight(const v2d &anchor, const v2d &dir) {
			p = anchor; d = dir; d.normalize();
		}

		v2d intersect(const Straight &s) const;
		float dist(const v2d &p) const;
		bool opposite(const v2d &a, const v2d &b);

		v2d p;
		v2d d;
};

inline v2d& v2d::operator=(const v2d &src) {
	x = src.x; y = src.y;
	return *this;
}

inline v2d v2d::operator+(const v2d &src) const {
	return v2d(x + src.x, y + src.y);
}

inline v2d v2d::operator-(void) const {
	return v2d(-x, -y);
}

inline v2d v2d::operator-(const v2d &src) const {
	return v2d(x - src.x, y - src.y);
}

inline v2d v2d::operator*(const float s) const {
	return v2d(s * x, s * y);
}

inline float v2d::operator*(const v2d &src) const {
	return src.x * x + src.y * y;
}

inline v2d operator*(const float s, const v2d &src) {
	return v2d(s * src.x, s * src.y);
}

inline float v2d::cosalpha(const v2d &p2, const v2d &c) const {
	v2d l1 = *this - c;
	v2d l2 = p2 - c;
	return (l1 * l2) / (l1.len() * l2.len());
}

inline v2d v2d::rotate(const v2d &c, float arc) const {
	v2d d = *this - c;
	float sina = sin(arc); float cosa = cos(arc);
	return c + v2d(d.x * cosa - d.y * sina, d.x * sina + d.y * cosa);
}

inline float v2d::len(void) const {
	assert((x*x + y*y) >= 0);
	return sqrt((x * x) + (y * y));
}

inline float v2d::dist(const v2d &p) const {
	return sqrt((p.x - x)*(p.x - x) + (p.y - y)*(p.y - y));
}

inline void v2d::normalize(void) {
	float l = this->len();
	x /= l;
	y /= l;
}

inline v2d Straight::intersect(const Straight &s) const {
	float t = -(d.x*(s.p.y - p.y) + d.y*(p.x - s.p.x))/(d.x*s.d.y - d.y*s.d.x);
	return s.p + s.d*t;
}

inline float Straight::dist(const v2d &s) const {
	v2d d1 = s - p;
	v2d d3 = d1 - d*d1*d;
	return d3.len();
}

inline bool Straight::opposite(const v2d &a, const v2d &b) {
	Straight s = Straight(a, b - a); 
	
	v2d x = this->intersect(s);

	float d1 = x.dist(a);
	float d2 = x.dist(b);
	float d3 = a.dist(b);

	float delta = d1 + d2 - d3;
	if (delta < 0) delta *= -1;

	if (delta < 0.001) return true;
	return false;
}

#endif

