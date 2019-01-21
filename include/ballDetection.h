#ifndef BALLDETECTION_H_
#define BALLDETECTION_H_

struct values {
	float angle;
	float radius;
};

struct morevalues {
	float angles[5];
	float radiuses[5];
};

struct values scan(float angle, float radius);
struct values single_scan(int sleep_value, 
		int scan_id, 
		float radius_value, 
		int sign, 
		float ignore_angle);
float theoretical_radius(float angle);

#endif
