#include "Flock.h"
#include "Vector3.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <gl\glut.h>				// GLUT
#include <gl\gl.h>				// GL
//#include <gl\glu.h>
//#include <gl\glaux.h>

void draw_pavilion();

Flock::Flock( int num_boids, double scene_radius, double influence_radius )
{
	_num_boids = num_boids;
	_scene_radius = scene_radius;
	_influence_radius = influence_radius;

	_boids = NULL;
	_obstacles = NULL;
	_dists = NULL;

	_min_dist = 0.00001;
}

Flock::~Flock()
{
	if( _boids )
		delete [] _boids;

	if( _obstacles )
		delete [] _obstacles;

	if( _dists )
	{
		for( int i = 0; i < _num_boids; ++i )
			delete [] _dists[i];
		delete [] _dists;
	}
}

void Flock::init()
{
	// 参数
	CollisionFraction = 0.8f;							// collision fraction
	InvCollisionFraction = 1.0/(1.0-CollisionFraction);

	_separation_scale	= 1.0;							// separation scale
	_alignment_scale	= 10;							// alignment scale
	_cohesion_scale		= 1 / _separation_scale;		// cohesion scale
	_migratory_scale	= 1;							// migratory scale
	_obstacle_scale		= 1;							// obstacle scale

	_separation_flag = true;
	_alignment_flag  = true;
	_cohesion_flag   = true;
	_migratory_flag  = true;
	_obstacle_flag   = true;

	_render_force = false;
	_render_dir   = false;

	 // 随机数种子
    srand( time( NULL ) );

    // 初始化boids
    _boids = new Boid[ _num_boids ];
	for( int i = 0; i < _num_boids; i++ )
		_boids[i].init( 0.05f * _scene_radius );
	
	// 初始化距离矩阵
	_dists = new double*[ _num_boids ];
    for( int i = 0; i < _num_boids; i++ )		// 分配距离矩阵的第二维
        _dists[i] = new double[_num_boids];

	// 初始化目标
	_goal = Vector3d( 0.0, 0.0, 0.0 );
	_goal_rad = 0.0;
	
	// 初始化障碍物
    _num_obstacles = 1;
    _obstacles = new Obstacle[_num_obstacles];
    _obstacles[0]._pos = Vector3d( 0, -10, 0 );
//   _obstacles[1]._pos = Vector3d( 0, 0, _scene_radius );
//    _obstacles[2]._pos = Vector3d( -_scene_radius, 0, 0 );
//    _obstacles[3]._pos = Vector3d( 0, 0, -_scene_radius );
    _obstacles[0]._radius = 6;
//    _obstacles[1]._radius = 1;
//    _obstacles[2]._radius = 5;
//    _obstacles[3]._radius = 2;
	_obstacles[0]._rot = 0;
//	_obstacles[1]._rot = 0;
//	_obstacles[2]._rot = 0;
//	_obstacles[3]._rot = 0;
//	_obstacles[0]._color = Vector3d( 1, 1, 0.5 );
//	_obstacles[1]._color = Vector3d( 1, 1, 0.5 );
//	_obstacles[2]._color = Vector3d( 1, 1, 0.5 );
//	_obstacles[3]._color = Vector3d( 1, 1, 0.5 );

	// 设定线粗
	glLineWidth( 3.0 );
}

void Flock::update( double sec )
{
	int i, j;
	Vector3d vec;
	double dist, sqr_dist, inv_dist, inv_sqr_dist;
	double dot;
	
	// 目标位置
	_goal_rad -= 3.0 * sec / _scene_radius;
	_goal = Vector3d( sin( _goal_rad ),  0.0, cos( _goal_rad ) ) * _scene_radius;
	
	// 计算相互距离，初始化各受力
	for( i = 0; i < _num_boids; i++ )
	{
		// 计算两两之间距离
		for( j = i + 1; j < _num_boids; j++ )
		{
			vec = _boids[i]._pos - _boids[j]._pos;					// 两两之间距离
			_dists[i][j] = _dists[j][i] = vec.length();				// 记录下来
		}
		_dists[i][i] = 0.0;											// 自己到自己的距离

		// 各种力复零
		_boids[i]._separation_force = Vector3d( 0.0, 0.0, 0.0 );
		_boids[i]._alignment_force  = Vector3d( 0.0, 0.0, 0.0 );
		_boids[i]._cohesion_force   = Vector3d( 0.0, 0.0, 0.0 );
		_boids[i]._migratory_force  = Vector3d( 0.0, 0.0, 0.0 );
		_boids[i]._obstacle_force   = Vector3d( 0.0, 0.0, 0.0 );

		// 邻居数目复零
		_boids[i]._num_neighbors = 0;
	}

	// 累计各种受力
	for( i = 0; i < _num_boids; i++ )
	{
		// 计算相互施加的影响力
		for( j = i + 1; j < _num_boids; j++ )
		{
			// 相互距离
			vec = _boids[i]._pos - _boids[j]._pos;					// 从j指向i的向量
			dist = vec.length();		// _dists[i][j];
			if( dist < _min_dist )
				dist = _min_dist;
			sqr_dist = dist * dist;
			inv_dist = 1.0 / dist;
			inv_sqr_dist = 1.0 / sqr_dist;
			
			// 若小于影响半径
			if( dist < _influence_radius )
			{
				// 累加 seperation force，系数为距离平方的倒数
				_boids[i]._separation_force += vec * inv_sqr_dist * _separation_scale;
				_boids[j]._separation_force -= vec * inv_sqr_dist * _separation_scale;

				// 累加 alignment force，系数为距离平方的倒数（计算邻居的累加方向，后面再计算平均值）
				_boids[i]._alignment_force += _boids[j]._dir * inv_dist;
				_boids[j]._alignment_force += _boids[i]._dir * inv_dist;

				// 累加 cohesion force，系数为1（计算邻居的累加坐标，后面再计算平均值）
				_boids[i]._cohesion_force += _boids[j]._pos;
				_boids[j]._cohesion_force += _boids[i]._pos;

				// 累计邻居数目
				_boids[i]._num_neighbors++;
				_boids[j]._num_neighbors++;
			}
		}

		// alignment force，计算邻居的平均方向
		if( _boids[i]._num_neighbors != 0 )
		{
			_boids[i]._alignment_force /= (double)_boids[i]._num_neighbors;	// 邻居的平均方向
			dist = _boids[i]._alignment_force.length();						// 平均方向的长度
			if( dist > _min_dist )
			{
				_boids[i]._alignment_force /= dist;							// 平均方向归一化
				vec = _boids[i]._alignment_force - _boids[i]._dir;			// 归一化平均方向 - 方向
				_boids[i]._alignment_force = vec * dist * _alignment_scale;
			} 
		}

		// cohesion force，计算邻居的平均坐标
		if( _boids[i]._num_neighbors )
		{
			_boids[i]._cohesion_force /= (double)_boids[i]._num_neighbors;	// 计算邻居的平均坐标
			vec = _boids[i]._cohesion_force - _boids[i]._pos;				// 本boid到邻居中心的向量
			_boids[i]._cohesion_force = vec * _cohesion_scale;
		}

		// obstacle force，加上障碍物的阻力
		for( j = 0; j < _num_obstacles; j++ )
		{
			vec = _boids[i]._pos - _obstacles[j]._pos;						// 从障碍物j指向boid的向量
			
			// 忽略已经飞过的障碍物（障碍物在后方）
			dot = vec * _boids[i]._dir;
			if( dot < 0.0 )
			{
				dist = vec.length() - _obstacles[j]._radius * 1.5;			// 计算障碍物的影响半径
				if( dist < _influence_radius )
				{
					if( dist < _min_dist )
						dist = _min_dist;
					vec.normalize();
					_boids[i]._obstacle_force += vec * (-dot * _obstacle_scale / dist);
				}
			}
		}

		// migratory force
		_boids[i]._migratory_force  = (_goal - _boids[i]._pos) * _migratory_scale;

		// 计算合力
		_boids[i]._force = Vector3d( 0 );
		if( _separation_flag )
			_boids[i]._force +=	_boids[i]._separation_force;
		if( _alignment_flag )
			_boids[i]._force +=	_boids[i]._alignment_force;
		if( _cohesion_flag )
			_boids[i]._force +=	_boids[i]._cohesion_force;
		if( _migratory_flag )
			_boids[i]._force +=	_boids[i]._migratory_force;
		if( _obstacle_flag )
			_boids[i]._force +=	_boids[i]._obstacle_force;

		// 若过大，则缩短，限于( -1-pi/2, 1+pi/2 )
		double mag = _boids[i]._force.length();
		if( mag > 1.0 )
			_boids[i]._force *= ( ( 1.0 + atan( mag - 1.0 ) ) / mag );

		// 更新所有boid
		_boids[i].update( sec );
	}

	/*double avg_speed = 0;
	for( i = 0; i < _num_boids; i++ )
		avg_speed += _boids[i]._speed;
	avg_speed /= _num_boids;
	printf("avg speed = %f\n", avg_speed );*/


	// 更新障碍物参数 /////////////////////////////////////
	for( int i = 0; i < _num_obstacles; i++ )
		_obstacles[i]._rot += 10 * sec;		// 10度/秒

	// 更新所有boids的中心 ////////////////////////////////
	_center = Vector3d( 0.0, 0.0, 0.0 );
    for( int i = 0; i < _num_boids; i++ )
        _center += _boids[i]._pos;
    _center /= (double)_num_boids;

	// 更新摄像机位置 /////////////////////////////////////
	_cam = _center
		+ Vector3d(sinf(-1 * _goal_rad), 0.7f + 0.75 * sinf(-1 * _goal_rad), cosf(-1 * _goal_rad))
		* 20.0;
	//_cam.y() = 5.0;
}

void Flock::render()
{
	// 绘制集群
	for( int i = 0; i < _num_boids; i++ )
	{
		

		glPushMatrix();

		if( _render_force )
		{
			glColor3f( 1, 1, 1 );
			glBegin( GL_LINES );
			glVertex3f( _boids[i]._last_pos.x(), _boids[i]._last_pos.y(), _boids[i]._last_pos.z() );
			glVertex3f( _boids[i]._last_pos.x() + _boids[i]._force.x(), 
				_boids[i]._last_pos.y() + _boids[i]._force.y(), 
				_boids[i]._last_pos.z() + _boids[i]._force.z() );
			glEnd();
		}

		glColor3f( _boids[i]._color.x(), _boids[i]._color.y(), _boids[i]._color.z() );
		if( _render_dir )
		{
			glBegin( GL_LINES );
			glVertex3f( _boids[i]._last_pos.x(), _boids[i]._last_pos.y(), _boids[i]._last_pos.z() );
			glVertex3f( _boids[i]._last_pos.x() + _boids[i]._dir.x(), 
				_boids[i]._last_pos.y() + _boids[i]._dir.y(), 
				_boids[i]._last_pos.z() + _boids[i]._dir.z() );
			glEnd();
		}
		glMultMatrixd( _boids[i]._tf );
		_boids[i].render();

		glPopMatrix();
	}

	// 绘制障碍物
	for( int i = 0; i < _num_obstacles; i++ )
	{
		glColor3d( _obstacles[i]._color.x(), _obstacles[i]._color.y(), _obstacles[i]._color.z() );
		glPushMatrix();
		glTranslated( _obstacles[i]._pos.x(), _obstacles[i]._pos.y(), _obstacles[i]._pos.z() );
		glRotated( -90.0, 1.0, 0.0, 0.0 );
		glRotated( _obstacles[i]._rot, 0.0, 0.0, 1.0 );
		glutWireSphere( _obstacles[i]._radius, 10, 5 );
		glPopMatrix();
	}

	draw_pavilion();
	// 绘制目标
	glColor3d( 1, 0.5, 0.5 );
	glPushMatrix();
	glTranslated( _goal.x(), _goal.y(), _goal.z() );
	glRotated( -90.0, 1.0, 0.0, 0.0 );
	glutWireIcosahedron();
	glPopMatrix();
}

void Flock::keyboard_char( unsigned char key, int x, int y ) {
	switch ( key ) {
	case '1':
		_separation_flag = !_separation_flag;
		printf("Separation Force: %s\n", "Off\0On" + 4 * _separation_flag);
		break;

	case '2':
		_cohesion_flag = !_cohesion_flag;
		printf("Cohesion Force: %s\n", "Off\0On" + 4 * _cohesion_flag);
		break;

	case '3':
		_alignment_flag = !_alignment_flag;
		printf("Alignment Force: %s\n", "Off\0On" + 4 * _alignment_flag);
		break;

	case '4':
		_migratory_flag = !_migratory_flag;
		printf("Migratory Force: %s\n", "Off\0On" + 4 * _migratory_flag);
		break;

	case '5':
		_obstacle_flag = !_obstacle_flag;
		printf("Obstacle Force: %s\n", "Off\0On" + 4 * _obstacle_flag);
		break;

	case '-':
		_render_force = !_render_force;
		break;

	case '=':
		_render_dir = !_render_dir;
		break;

	default:
		break;
	}
}

void Flock::keyboard_int( int a_keys, int x, int y )
{
}

void Flock::mouse_buttons(int button, int state, int x, int y)
{
}

void Flock::mouse_moving( int x, int y )
{
}

void draw_pavilion() {
	float scale = 1.0f;
	double h = -1.0f;		//坐标系整体上移h
	double s = 10.0f;

	glLineWidth(6.0f);

	//屋顶第一层
	glPushMatrix();
	glColor3f(1, 0, 0);
	glTranslated(0.0f, h*s, 0.0f);
	glRotatef(-90, 1, 0, 0);
	glutSolidCone(0.6*s, 0.25*s, 4, 3);
	glPopMatrix();

	//屋顶第二层
	glPushMatrix();
	glColor3f(0, 1, 0);
	glTranslated(0.0f, (0.1f + h)*s, 0.0f);
	glRotatef(-90, 1, 0, 0);
	glutSolidCone(0.42*s, 0.2*s, 4, 3);
	glPopMatrix();

	//屋顶第三层
	glPushMatrix();
	glColor3f(0, 0, 1);
	glTranslated(0.0f, (0.2f + h)*s, 0.0f);
	glRotatef(-90, 1, 0, 0);
	glutSolidCone(0.25*s, 0.15*s, 4, 3);
	glPopMatrix();

	//屋顶圆球
	glPushMatrix();
	glColor3f(1, 0, 1);
	glTranslated(0.0f, (0.37f + h)*s, 0.0f);
	glRotatef(-90, 1, 0, 0);
	glutSolidSphere(0.05*s, 10, 10);
	glPopMatrix();

	//屋檐
	GLUquadricObj *cylinder_obj1;
	GLUquadricObj *cylinder_obj2;
	cylinder_obj1 = gluNewQuadric();
	cylinder_obj2 = gluNewQuadric();
	glPushMatrix();
	glColor3f(1, 1, 0);
	glTranslated(0.0f, (0.34f + h)*s, 0.0f);
	glRotatef(31, 1, 0, 0);
	gluCylinder(cylinder_obj1, 0.02*s, 0.02*s, 0.7*s, 10, 10);
	glTranslated(0.0f, 0.0f, 0.7f*s);
	gluDisk(cylinder_obj2, 0, 0.02*s, 10, 10);
	glPopMatrix();

	cylinder_obj1 = gluNewQuadric();
	cylinder_obj2 = gluNewQuadric();
	glPushMatrix();
	glColor3f(1, 1, 0);
	glTranslated(0.0f, (0.34f + h)*s, 0.0f);
	glRotatef(149, 1, 0, 0);
	gluCylinder(cylinder_obj1, 0.02*s, 0.02*s, 0.7*s, 10, 10);
	glTranslated(0.0f, 0.0f, 0.7f*s);
	gluDisk(cylinder_obj2, 0, 0.02*s, 10, 10);
	glPopMatrix();

	cylinder_obj1 = gluNewQuadric();
	cylinder_obj2 = gluNewQuadric();
	glPushMatrix();
	glColor3f(1, 1, 0);
	glTranslated(0.0f, (0.34f + h)*s, 0.0f);
	glRotatef(90, 0, 1, 0);
	glRotatef(149, 1, 0, 0);
	gluCylinder(cylinder_obj1, 0.02*s, 0.02*s, 0.7*s, 10, 10);
	glTranslated(0.0f, 0.0f, 0.7f*s);
	gluDisk(cylinder_obj2, 0, 0.02*s, 10, 10);
	glPopMatrix();

	cylinder_obj1 = gluNewQuadric();
	cylinder_obj2 = gluNewQuadric();
	glPushMatrix();
	glColor3f(1, 1, 0);
	glTranslated(0.0f, (0.34f + h)*s, 0.0f);
	glRotatef(-90, 0, 1, 0);
	glRotatef(149, 1, 0, 0);
	gluCylinder(cylinder_obj1, 0.02*s, 0.02*s, 0.7*s, 10, 10);
	glTranslated(0.0f, 0.0f, 0.7f*s);
	gluDisk(cylinder_obj2, 0, 0.02*s, 10, 10);
	glPopMatrix();

	//支柱
	cylinder_obj1 = gluNewQuadric();
	cylinder_obj2 = gluNewQuadric();
	glPushMatrix();
	glColor3f(0, 1, 1);
	glTranslated(0.0f, (0.0f + h)*s, 0.55f*s);
	glRotatef(90, 1, 0, 0);
	gluCylinder(cylinder_obj1, 0.03*s, 0.03*s, 0.7*s, 10, 10);
	glTranslated(0.0f, 0.0f, 0.7f*s);
	gluDisk(cylinder_obj2, 0, 0.03*s, 10, 10);
	glPopMatrix();

	cylinder_obj1 = gluNewQuadric();
	cylinder_obj2 = gluNewQuadric();
	glPushMatrix();
	glColor3f(0, 1, 1);
	glTranslated(0.0f, (0.0f + h)*s, -0.55f*s);
	glRotatef(90, 1, 0, 0);
	gluCylinder(cylinder_obj1, 0.03*s, 0.03*s, 0.7*s, 10, 10);
	glTranslated(0.0f, 0.0f, 0.7f*s);
	gluDisk(cylinder_obj2, 0, 0.03*s, 10, 10);
	glPopMatrix();

	cylinder_obj1 = gluNewQuadric();
	cylinder_obj2 = gluNewQuadric();
	glPushMatrix();
	glColor3f(0, 1, 1);
	glTranslated(0.55f*s, (0.0f + h)*s, 0.0f);
	glRotatef(90, 1, 0, 0);
	gluCylinder(cylinder_obj1, 0.03*s, 0.03*s, 0.7*s, 10, 10);
	glTranslated(0.0f, 0.0f, 0.7f*s);
	gluDisk(cylinder_obj2, 0, 0.03*s, 10, 10);
	glPopMatrix();

	cylinder_obj1 = gluNewQuadric();
	cylinder_obj2 = gluNewQuadric();
	glPushMatrix();
	glColor3f(0, 1, 1);
	glTranslated(-0.55f*s, h*s, 0.0f);
	glRotatef(90, 1, 0, 0);
	gluCylinder(cylinder_obj1, 0.03*s, 0.03*s, 0.7*s, 10, 10);
	glTranslated(0.0f, 0.0f, 0.7f*s);
	gluDisk(cylinder_obj2, 0, 0.03*s, 10, 10);
	glPopMatrix();

	//底座
	glPushMatrix();
	glColor3f(1, 0, 1);
	glTranslated(0.0f, (-0.75f + h)*s, 0.0f);
	glRotatef(45, 0, 1, 0);
	glScalef(1, 0.1, 1);
	glutSolidCube(1*s);
	glPopMatrix();

	//桌面
	glPushMatrix();
	glColor3f(0, 0, 1);
	glTranslated(0.0f,( -0.5f + h)*s, 0.0f);
	glRotatef(45, 0, 1, 0);
	glScalef(1, 0.03, 1);
	glutSolidCube(0.4*s);
	glPopMatrix();

	//桌脚
	cylinder_obj1 = gluNewQuadric();
	cylinder_obj2 = gluNewQuadric();
	glPushMatrix();
	glColor3f(0, 1, 0);
	glTranslated(-0.23f*s, (-0.5f + h)*s, 0.0f);
	glRotatef(90, 1, 0, 0);
	gluCylinder(cylinder_obj1, 0.01*s, 0.01*s, 0.23*s, 10, 10);
	glTranslated(0.0f, 0.0f, 0.23f*s);
	gluDisk(cylinder_obj2, 0, 0.01*s, 10, 10);
	glPopMatrix();

	cylinder_obj1 = gluNewQuadric();
	cylinder_obj2 = gluNewQuadric();
	glPushMatrix();
	glColor3f(0, 1, 0);
	glTranslated(0.23f*s, (-0.5f + h)*s, 0.0f);
	glRotatef(90, 1, 0, 0);
	gluCylinder(cylinder_obj1, 0.01*s, 0.01*s, 0.23*s, 10, 10);
	glTranslated(0.0f, 0.0f, 0.23f*s);
	gluDisk(cylinder_obj2, 0, 0.01*s, 10, 10);
	glPopMatrix();

	cylinder_obj1 = gluNewQuadric();
	cylinder_obj2 = gluNewQuadric();
	glPushMatrix();
	glColor3f(0, 1, 0);
	glTranslated(0.0f, (-0.5f + h)*s, 0.23f*s);
	glRotatef(90, 1, 0, 0);
	gluCylinder(cylinder_obj1, 0.01*s, 0.01*s, 0.23*s, 10, 10);
	glTranslated(0.0f, 0.0f, 0.23f*s);
	gluDisk(cylinder_obj2, 0, 0.01*s, 10, 10);
	glPopMatrix();

	cylinder_obj1 = gluNewQuadric();
	cylinder_obj2 = gluNewQuadric();
	glPushMatrix();
	glColor3f(0, 1, 0);
	glTranslated(0.0f, (-0.5f + h)*s, -0.23f*s);
	glRotatef(90, 1, 0, 0);
	gluCylinder(cylinder_obj1, 0.01*s, 0.01*s, 0.23*s, 10, 10);
	glTranslated(0.0f, 0.0f, 0.23f*s);
	gluDisk(cylinder_obj2, 0, 0.01*s, 10, 10);
	glPopMatrix();

	//茶壶
	glPushMatrix();
	glColor3f(1, 1, 0);
	glTranslated(0.0f, (-0.45f + h)*s, 0.0f);
	glutSolidTeapot(0.05*s);
	glPopMatrix();

	//灯座
	glPushMatrix();
	glColor3f(0.5, 0.5, 0.5);
	glTranslated(0.0f, -0.0f + h*s, 0.0f);
	glRotatef(90, 1, 0, 0);
	glutSolidCone(0.05*s, 0.05*s, 4, 3);
	glPopMatrix();

	//灯
	glPushMatrix();
	glColor3f(1, 1, 1);
	glTranslated(0.0f, (-0.07f + h)*s, 0.0f);
	glScalef(0.05*s, 0.05*s, 0.05*s);
	glutSolidIcosahedron();
	glPopMatrix();
}