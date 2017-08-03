#pragma once

#include "Vector3.h"
#include "Boid.h"

#include <math.h>

struct Obstacle					// 障碍物
{
	double   _radius;			// 半径
	double   _rot;				// 旋转量
	Vector3d _pos;				// 位置
	Vector3d _color;			// 颜色
};

class Flock						// 集群
{
public:
	Flock( int num_boids = 100, double scene_radius = 10.0, double influence_radius = 1.0 );
	~Flock();

public:
	double _scene_radius;		// 场景半径
	double _influence_radius;	// 邻居影响半径
	Vector3d _center;			// 集群中心
	Vector3d _cam;				// 摄像机位置
	
	int _num_boids;				// boid数目
	Boid *_boids;				// boid数组

	double** _dists;			// 2-d array of boid distances, yuk what a waste
	
	int _num_obstacles;			// 障碍物数目
	Obstacle *_obstacles;		// 障碍物数组
	
	Vector3d _goal;				// 目标
	double _goal_rad;			// 目标公转角度

	double _min_dist;			// 最小距离
public:
	
	double CollisionFraction;		// collision fraction
	double InvCollisionFraction;

	// More arbitray constants that look cool
	double	_separation_scale;	// separation scale
	double	_alignment_scale;		// alignment scale
	double	_cohesion_scale;		// cohesion scale
	double	_migratory_scale;		// migratory scale
	double	_obstacle_scale;		// obstacle scale

	bool _separation_flag;
	bool _alignment_flag;
	bool _cohesion_flag;
	bool _migratory_flag;
	bool _obstacle_flag;
	
	bool _render_force;
	bool _render_dir;

public:
	void init();
	void update( double sec );
	void render();
	void keyboard_char( unsigned char key, int x, int y );
	void keyboard_int( int a_keys, int x, int y );
	void mouse_buttons(int button, int state, int x, int y);
	void mouse_moving( int x, int y );
};

