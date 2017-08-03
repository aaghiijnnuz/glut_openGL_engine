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

// scatter_range: ����ֲ�����������ı߳���������������ԭ�㡣
void Boid::init( double scatter_range )
{
	// �ؼ�����
	_normal_speed = 3;							// �����ٶȣ�n��λ����/��
	_max_speed = _normal_speed * 5;				// ����ٶ�
	_min_speed = _normal_speed * 0.5;			// ��С�ٶ�
	_ratio_force_to_pitch = 60;					// n��/��/��λ��
	_ratio_pitch_to_speed = 0.01;				// n��λ����/��/��
	_ratio_force_to_roll = 30;					// n��/��/��λ��
	_ratio_force_to_speed = 0.1;				// n��λ����/��^2/��λ��

	// ��ʼ����ɫ
	//_color  = Vector3d( 0.5f + 0.5 * rndf(), 0.5f + 0.5 * rndf(), 0.5f + 0.5 * rndf() );
	_color  = Vector3d( rndf(), rndf(), rndf() );
	double min_val = _color.min_component( _color );
	double max_val = _color.max_component( _color );
	_color -= Vector3d( min_val, min_val, min_val );
	_color /= (max_val - min_val);
	_color = _color * 0.7 + Vector3d( 0.3, 0.3, 0.3 );

	// ����
	_type = rndf();

	// ��С
	_size = 1.0;	// 0.5 + rndf() * 1.5;
	
	// ��ʼ�� _tf=I
	for( int i = 0; i < 16; ++i )
		_tf[i] = 0.0f;
	_tf[ 0] = 1.0f;
	_tf[ 5] = 1.0f;
	_tf[10] = 1.0f;
	_tf[15] = 1.0f;
	
	// ��ʼ��λ��
	_pos = Vector3d( rndf() - rndf(), 
		rndf() - rndf() , 
		rndf() - rndf() ) * (0.5 * scatter_range);

	// ��ʼ���ٶȵȲ���
	// ��Ӧ���ݸ����Ȳ������㣬��������update��Ȼ����¸��ֲ�����
	_dir = Vector3d( rndf() - rndf(), rndf() - rndf(), rndf() - rndf());
	_dir.normalize();					// ��һ��
	_speed = _normal_speed;
	_pitch = _yaw = _roll = 0.0f;
}

void Boid::update( double sec )
{
	///////////////////////////////////////////////////
	// �������

	Vector3d left = Vector3d(0, 1, 0) % _dir;	// ��λ�������˶��������
	Vector3d up   = _dir % left;				// �ϵ�λ�������˶������ϲ�
	
	double dot_dir  = _force * _dir;			// ǰ������С
	double dot_up   = _force * up;				// �Ϸ�����С
	double dot_left = _force * left;			// �������С

	Vector3d force_dir  = _dir * dot_dir;		// ǰ����
	Vector3d force_up   = up   * dot_up;		// �Ϸ���
	Vector3d force_left = left * dot_left;		// �����
	

	///////////////////////////////////////////////////
	// �����仯
	
	double dp = _ratio_force_to_pitch * sec * _force.y();	// �����Ǻ�����y����
	//double dp = _ratio_force_to_pitch * sec * dot_up;		// ����n��/��/��λ��
	_pitch += dp;
	_pitch = _pitch > 45 ? 45 : (_pitch < -45 ? -45 : _pitch);
	_pitch *= 0.98;
	//_pitch *= std::max(1.0 - 0.1 * sec, 0);						// ����
	
	double angle = _pitch;
	if( angle > 30 )
		angle = 30;
	_speed -= _speed * _ratio_pitch_to_speed * angle * sec;	// ���ݸ������������ٶ�
	
	///////////////////////////////////////////////////
	// �����ƫ�������ң�

	Vector3d hforce( _force.x(), 0, _force.z() );					// ������ˮƽ���ͶӰ��ˮƽ��
	if( hforce.lengthSquared() > 0.00001 )							// ��ˮƽ����Ϊ��
	{
		//Vector3d dir_force = Vector3d( _dir.x(), 0, _dir.z() );		// �ٶ���ˮƽ���ͶӰ��ˮƽ�ٶ�
		Vector3d dir_force = _dir;
		//if( dir_force.lengthSquared() > 0.00001 )
		//{
			//dir_force.normalize();									// ˮƽ�ٶȷ���
			Vector3d right_vec = dir_force % Vector3d( 0, 1, 0 );
			double len_dir_force = hforce * dir_force;				// ˮƽ����ˮƽ�ٶȷ����ϵ�ͶӰ��ǰ��������
			dir_force *= len_dir_force;								// ǰ����

			if( len_dir_force > 0 )
				_speed += _ratio_force_to_speed * len_dir_force * sec;	// ����ˮƽ�����ʵ�����


			double len_side_force = right_vec * hforce;			// ����������
			//right_vec *= len_side_force;							// ������
			
			double droll = _ratio_force_to_roll * len_side_force * sec;	// ����n��/��/��λ������������
			_roll += droll;											// �������������
			_roll = _roll > 75 ? 75 : (_roll < -75 ? -75 : _roll);	// �������
			_roll *= 0.95;
		//}
	}
	_yaw -= 0.05 * _roll;													// ƫ��
	
	// �����ٶ� ///////////////////////////////////////

	_speed = _speed > _max_speed ? _max_speed : 
		(_speed < _min_speed ? _min_speed : _speed);		// �ٶ�����

	double a = std::max(1.0 - 0.02 * sec, 0.0);
	_speed = (_speed * a + _normal_speed * (1 - a));		// ������ᣬ�𽥻ָ��������ٶ�
	//_speed = _speed * 0.99 + _normal_speed * 0.01;
	
	// ������λ�� /////////////////////////////////////

	//_pos += _dir * ( _speed * sec * 30 );
	
	
	// �����ٶȷ��� ///////////////////////////////////

	// ����ֲ�����ϵ-��������ϵת������
	glPushMatrix();
	glLoadIdentity();
	glTranslated( _pos.x(), _pos.y(), _pos.z() );
	glRotated( _yaw,   0.0f, 1.0f, 0.0f );			// ƫ��
	glRotated( _pitch, 0.0f, 0.0f, 1.0f );			// ����
	glRotated( _roll,  1.0f, 0.0f, 0.0f );			// ���
	glGetDoublev( GL_MODELVIEW_MATRIX, _tf ); 
	glPopMatrix();

	// �����ٶȷ��򣬼��ֲ�����ϵ��x�ᣨ��ͷ�ĳ�������������ϵ�ķ���
	_dir.x() = _tf[0];
	_dir.y() = _tf[1];
	_dir.z() = _tf[2];

	// ������λ�� /////////////////////////////////////

	_last_pos = _pos;
	_pos += _dir * ( _speed * sec );
}

void Boid::render() {
	if( _type > 0.5 )
		draw_plane( 2 );
	else
		draw_helicopter( 1 );
}

