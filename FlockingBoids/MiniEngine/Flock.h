#pragma once

#include "Vector3.h"
#include "Boid.h"

#include <math.h>

struct Obstacle					// �ϰ���
{
	double   _radius;			// �뾶
	double   _rot;				// ��ת��
	Vector3d _pos;				// λ��
	Vector3d _color;			// ��ɫ
};

class Flock						// ��Ⱥ
{
public:
	Flock( int num_boids = 100, double scene_radius = 10.0, double influence_radius = 1.0 );
	~Flock();

public:
	double _scene_radius;		// �����뾶
	double _influence_radius;	// �ھ�Ӱ��뾶
	Vector3d _center;			// ��Ⱥ����
	Vector3d _cam;				// �����λ��
	
	int _num_boids;				// boid��Ŀ
	Boid *_boids;				// boid����

	double** _dists;			// 2-d array of boid distances, yuk what a waste
	
	int _num_obstacles;			// �ϰ�����Ŀ
	Obstacle *_obstacles;		// �ϰ�������
	
	Vector3d _goal;				// Ŀ��
	double _goal_rad;			// Ŀ�깫ת�Ƕ�

	double _min_dist;			// ��С����
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

