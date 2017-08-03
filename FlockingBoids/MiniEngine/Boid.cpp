#include "Boid.h"

#include "Vector3.h"
#include "Geometries.h"

#include <stdio.h>
#include <algorithm>

Boid::Boid()
{
}

Boid::~Boid()
{
}

// scatter_range: 随机分布所在立方体的边长。立方体中心在原点。
void Boid::init( double scatter_range )
{
	// 关键参数
	_normal_speed = 3;							// 正常速度，n单位距离/秒
	_max_speed = _normal_speed * 5;				// 最高速度
	_min_speed = _normal_speed * 0.5;			// 最小速度
	_ratio_force_to_pitch = 60;					// n度/秒/单位力
	_ratio_pitch_to_speed = 0.01;				// n单位距离/度/秒
	_ratio_force_to_roll = 30;					// n度/秒/单位力
	_ratio_force_to_speed = 0.1;				// n单位距离/秒^2/单位力

	// 初始化颜色
	//_color  = Vector3d( 0.5f + 0.5 * rndf(), 0.5f + 0.5 * rndf(), 0.5f + 0.5 * rndf() );
	_color  = Vector3d( rndf(), rndf(), rndf() );
	double min_val = _color.min_component( _color );
	double max_val = _color.max_component( _color );
	_color -= Vector3d( min_val, min_val, min_val );
	_color /= (max_val - min_val);
	_color = _color * 0.7 + Vector3d( 0.3, 0.3, 0.3 );

	// 外形
	_type = rndf();

	// 大小
	_size = 1.0;	// 0.5 + rndf() * 1.5;
	
	// 初始化 _tf=I
	for( int i = 0; i < 16; ++i )
		_tf[i] = 0.0f;
	_tf[ 0] = 1.0f;
	_tf[ 5] = 1.0f;
	_tf[10] = 1.0f;
	_tf[15] = 1.0f;
	
	// 初始化位置
	_pos = Vector3d( rndf() - rndf(), 
		rndf() - rndf() , 
		rndf() - rndf() ) * (0.5 * scatter_range);

	// 初始化速度等参数
	// 本应根据俯仰等参数计算，不过调用update自然会更新各种参数。
	_dir = Vector3d( rndf() - rndf(), rndf() - rndf(), rndf() - rndf());
	_dir.normalize();					// 归一化
	_speed = _normal_speed;
	_pitch = _yaw = _roll = 0.0f;
}

void Boid::update( double sec )
{
	///////////////////////////////////////////////////
	// 计算合力

	Vector3d left = Vector3d(0, 1, 0) % _dir;	// 左单位向量，运动方向左侧
	Vector3d up   = _dir % left;				// 上单位向量，运动方向上侧
	
	double dot_dir  = _force * _dir;			// 前分力大小
	double dot_up   = _force * up;				// 上分力大小
	double dot_left = _force * left;			// 左分力大小

	Vector3d force_dir  = _dir * dot_dir;		// 前分力
	Vector3d force_up   = up   * dot_up;		// 上分力
	Vector3d force_left = left * dot_left;		// 左分力
	

	///////////////////////////////////////////////////
	// 俯仰变化
	
	double dp = _ratio_force_to_pitch * sec * _force.y();	// 仅考虑合力的y分量
	//double dp = _ratio_force_to_pitch * sec * dot_up;		// 增减n度/秒/单位力
	_pitch += dp;
	_pitch = _pitch > 45 ? 45 : (_pitch < -45 ? -45 : _pitch);
	_pitch *= 0.98;
	//_pitch *= std::max(1.0 - 0.1 * sec, 0);						// 阻尼
	
	double angle = _pitch;
	if( angle > 30 )
		angle = 30;
	_speed -= _speed * _ratio_pitch_to_speed * angle * sec;	// 根据俯仰攻角增减速度
	
	///////////////////////////////////////////////////
	// 横滚和偏航（左右）

	Vector3d hforce( _force.x(), 0, _force.z() );					// 合力在水平面的投影，水平力
	if( hforce.lengthSquared() > 0.00001 )							// 若水平力不为零
	{
		//Vector3d dir_force = Vector3d( _dir.x(), 0, _dir.z() );		// 速度在水平面的投影，水平速度
		Vector3d dir_force = _dir;
		//if( dir_force.lengthSquared() > 0.00001 )
		//{
			//dir_force.normalize();									// 水平速度方向
			Vector3d right_vec = dir_force % Vector3d( 0, 1, 0 );
			double len_dir_force = hforce * dir_force;				// 水平力在水平速度方向上的投影，前向力长度
			dir_force *= len_dir_force;								// 前向力

			if( len_dir_force > 0 )
				_speed += _ratio_force_to_speed * len_dir_force * sec;	// 根据水平受力适当加速


			double len_side_force = right_vec * hforce;			// 侧向力长度
			//right_vec *= len_side_force;							// 侧向力
			
			double droll = _ratio_force_to_roll * len_side_force * sec;	// 增减n度/秒/单位力（左负右正）
			_roll += droll;											// 横滚（左负右正）
			_roll = _roll > 75 ? 75 : (_roll < -75 ? -75 : _roll);	// 横滚限制
			_roll *= 0.95;
		//}
	}
	_yaw -= 0.05 * _roll;													// 偏航
	
	// 计算速度 ///////////////////////////////////////

	_speed = _speed > _max_speed ? _max_speed : 
		(_speed < _min_speed ? _min_speed : _speed);		// 速度限制

	double a = std::max(1.0 - 0.02 * sec, 0.0);
	_speed = (_speed * a + _normal_speed * (1 - a));		// 添加阻尼，逐渐恢复至正常速度
	//_speed = _speed * 0.99 + _normal_speed * 0.01;
	
	// 计算新位置 /////////////////////////////////////

	//_pos += _dir * ( _speed * sec * 30 );
	
	
	// 计算速度方向 ///////////////////////////////////

	// 计算局部坐标系-世界坐标系转换矩阵
	glPushMatrix();
	glLoadIdentity();
	glTranslated( _pos.x(), _pos.y(), _pos.z() );
	glRotated( _yaw,   0.0f, 1.0f, 0.0f );			// 偏航
	glRotated( _pitch, 0.0f, 0.0f, 1.0f );			// 俯仰
	glRotated( _roll,  1.0f, 0.0f, 0.0f );			// 横滚
	glGetDoublev( GL_MODELVIEW_MATRIX, _tf ); 
	glPopMatrix();

	// 计算速度方向，即局部坐标系的x轴（机头的朝向）在世界坐标系的方向
	_dir.x() = _tf[0];
	_dir.y() = _tf[1];
	_dir.z() = _tf[2];

	// 计算新位置 /////////////////////////////////////

	_last_pos = _pos;
	_pos += _dir * ( _speed * sec );
}

void Boid::render() {
	if( _type > 0.5 )
		draw_plane( 2 );
	else
		draw_helicopter( 1 );
}

