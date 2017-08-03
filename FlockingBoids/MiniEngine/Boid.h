#pragma once

#include "Vector3.h"

class Boid
{
public:
	Boid();
	~Boid();
	
public:
	void init( double scatter_radius );
	void update( double msec );

public:
	double _normal_speed;			// 正常速度，单位距离/秒
	double _max_speed;				// 最高速度
	double _min_speed;				// 最小速度
	double _ratio_force_to_pitch;	// 度/秒/单位力
	double _ratio_pitch_to_speed;	// /度/秒
	double _ratio_force_to_roll;	// 度/秒/单位力
	double _ratio_force_to_speed;	// 度/秒/单位力

	Vector3d _color;				// 颜色
	double _type;					// 外形
	double _size;					// 大小

	double _tf[16];					// 局部坐标-世界坐标转换矩阵

	Vector3d _pos, _last_pos;		// 位置
	Vector3d _dir;					// 速度方向
	double _speed;					// 速度值
	double _pitch, _yaw, _roll;		// 方向参数
	
	Vector3d _separation_force;	// separation force
	Vector3d _alignment_force;	// alignment force
	Vector3d _cohesion_force;	// cohesion force
	Vector3d _migratory_force;	// migratory force
	Vector3d _obstacle_force;	// obstacle force
	Vector3d _force;				// 合力

	int      _num_neighbors;	// num of neighbors

	Vector3d vDeltaPos;			// change in position from flock centering
	Vector3d vDeltaDir;			// change in direction
	int iDeltaCnt;				// number of boids that influence this delta_dir
	
		
public:
	void render();
};

