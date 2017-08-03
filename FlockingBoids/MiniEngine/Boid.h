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
	double _normal_speed;			// �����ٶȣ���λ����/��
	double _max_speed;				// ����ٶ�
	double _min_speed;				// ��С�ٶ�
	double _ratio_force_to_pitch;	// ��/��/��λ��
	double _ratio_pitch_to_speed;	// /��/��
	double _ratio_force_to_roll;	// ��/��/��λ��
	double _ratio_force_to_speed;	// ��/��/��λ��

	Vector3d _color;				// ��ɫ
	double _type;					// ����
	double _size;					// ��С

	double _tf[16];					// �ֲ�����-��������ת������

	Vector3d _pos, _last_pos;		// λ��
	Vector3d _dir;					// �ٶȷ���
	double _speed;					// �ٶ�ֵ
	double _pitch, _yaw, _roll;		// �������
	
	Vector3d _separation_force;	// separation force
	Vector3d _alignment_force;	// alignment force
	Vector3d _cohesion_force;	// cohesion force
	Vector3d _migratory_force;	// migratory force
	Vector3d _obstacle_force;	// obstacle force
	Vector3d _force;				// ����

	int      _num_neighbors;	// num of neighbors

	Vector3d vDeltaPos;			// change in position from flock centering
	Vector3d vDeltaDir;			// change in direction
	int iDeltaCnt;				// number of boids that influence this delta_dir
	
		
public:
	void render();
};

