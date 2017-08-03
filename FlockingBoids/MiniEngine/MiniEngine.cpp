///////////////////////////////////////////////////////////////////////////////
// 名称: Mini 3D Engine
// 版本: 1.0.2013-06-11.2053
// 
// 作者: 李峥
// 电邮：lizheng AT gdut.edu.cn
// 单位：计算机学院，广东工业大学
//
// 本软件按照GNU GPLv3授权。参阅 http://www.gnu.org/copyleft/gpl.html。
// 广东工业大学版权所有，2013年。保留所有权利。
// 
// 用法：在命令行窗口输入以下命令
//        > MiniEngine [filename].obj
//
///////////////////////////////////////////////////////////////////////////////
//
// Project: Mini 3D Engine
// Version: 1.0.2013-06-12.1730
// 
// Author: LI Zheng
// E-Mail: lizheng AT gdut.edu.cn
// Affiliation: School of Computers, Guangdong University of Technology
//
// Licensed under the GNU GPLv3. See http://www.gnu.org/copyleft/gpl.html.
// Copyright: 2013 Guangdong University of Technology. All rights reserved.
// 
// Usage: In command line window, enter the following command
//        > MiniEngine [filename].obj
// 
///////////////////////////////////////////////////////////////////////////////
// 
//			Patience is a virtue.
//			Possess it if you can.
//			Seldom found in woman.
//			Never found in man.
// 
///////////////////////////////////////////////////////////////////////////////

// 头文件 /////////////////////////////////////////////////////////////////////

#include <windows.h>			// 窗口
#include <stdio.h>				// 输入输出
#include <vector>				// std数组
#include <time.h>				// 时间

//#include "Geometries.h"
#include "Flock.h"

#include <gl\glut.h>				// GLUT
#include <gl\gl.h>				// GL
#include <gl\glu.h>
//#include <gl\glaux.h>

using namespace std;			// 使用std命名空间，简化变量声明


// 全局参数 ///////////////////////////////////////////////////////////////////

double _camx = 0.0;				// 摄像机位置x
double _camy = 0.0;				// 摄像机位置y
double _camz = 3.0;				// 摄像机位置z
double _tgtx = 0.0;				// 摄像机目标x
double _tgty = 0.0;				// 摄像机目标y
double _tgtz = 0.0;				// 摄像机目标z

bool _fullscreen = false;		// 全屏显示
bool _wireframe = false;			// 线框渲染
bool _cull_back_faces = false;	// 去除背向面，提高渲染速度
bool _lighting = true;			// 光照
bool _smooth_shading = true;	// 光滑渲染

vector<double> _verts;			// 顶点坐标数组：x0, y0, z0, x1, y1, z1, ...
vector<unsigned int> _faces;	// 三角形顶点序号数组：v0x, v0y, v0z, v1x, v1y, v1z, ...
vector<double> _vert_normals;	// 顶点法线数组。每个顶点对应一条法线。nx0, ny0, nz0, nx1, ny1, nz1, ...
vector<double> _face_normals;	// 面法线数组。每个面对应一条法线。n0x, n0y, n0z, n1x, n1y, n1z, ...

double _scale = 1.0;			// 物体缩放
double _rot_x;					// 绕x轴的旋转角度
double _rot_y;					// 绕y轴的旋转角度

int _mouse_state = GLUT_UP;		// 鼠标按钮状态
int _mouse_button;				// 鼠标按钮
int _mouse_x, _mouse_y;			// 上次鼠标的屏幕坐标

clock_t _now, _justnow;			// 当前时间、刚才时间
clock_t _msec;					// 流逝时间（毫秒），可用于计算动画速度相关变量

clock_t _fps_time;				// 帧速测量起始时间
int _fps_interval = 1;			// 帧速测量间隔（秒）
int _fps_cnt = 0;				// 帧/秒

char _title[128];				// 窗口标题

const char _help[] = "Mini 3D Engine 1.0\n"
	"LI Zheng, lizheng AT gdut.edu.cn\n"
	"School of Computers, Guangdong University of Technology\n"
	"Licensed under the GNU GPLv3.\n" 
	"Copyright: 2013 Guangdong University of Technology. All rights reserved.\n\n"
	"Usage: MiniEngine [filename].obj\n"
	"Note that only a limited version of Wavefront Object File format is supported.\n"
	"arrow keys / left mouse button: rotate the model.\n"
	"w, s, a, d, q, e: pan the camera\n"
	"z, x: scale the model\n"
	"r: reset the camera\n"
	"h: print this help info\n"
	"Esc: exit the program\n"
	"F1: full screen / windowed mode\n"
	"F2: wireframe / surface mode\n"
	"F3: smooth / flat shading\n"
	"F4: cull back faces on / off\n"
	"F5: lighting on / off\n\n";

// Flock //////////////////////////////////////////////////////////////////////

Flock _flock( 100 );


// 函数 ///////////////////////////////////////////////////////////////////////

// 读取Wavefront Object File。
// 只支持有限的格式。
// 仅支持三角面顶点索引，不能处理法线、贴图坐标索引。
// 
// path:     文件路径
// verts:    顶点数组。
//           按照 x0, y0, z0, x1, y1, z1, x2, y2, z2, ... 每三个坐标值为顺序记录。
//           n个点有3*n个记录。
// faces:    三角面的顶点序号数组。
//           按照 f0_0, f0_1, f0_2, f1_0, f1_1, f1_2, f2_0, f2_1, f2_2, ... 每三个顶点序号为顺序记录。
//           n个三角形有3*n个记录。
bool load_obj_file( const char *path, vector<double> &verts, vector<unsigned int> &faces ){
	FILE *file = fopen( path, "r" );
	if( file == NULL )			// 找不到文件
		return false;

	while( true ) {
		char lineHeader[128];
		// 读取当前行第一个单词
		int res = fscanf(file, "%s", lineHeader);
		if (res == EOF)		// 到文件结尾，退出循环
			break;

		// 解析文档内容
		if ( strcmp( lineHeader, "v" ) == 0 ) {				// 顶点坐标
			double vx, vy, vz;
			fscanf(file, "%f %f %f\n", &vx, &vy, &vz );
			verts.push_back( vx );
			verts.push_back( vy );
			verts.push_back( vz );
		}else if ( strcmp( lineHeader, "f" ) == 0 ) {		// 面索引 <<仅支持三角面顶点索引，不能处理法线、贴图坐标索引！>>
			std::string vertex1, vertex2, vertex3;
			unsigned int vertexIndex[3];
			int matches = fscanf(file, "%d %d %d\n", &vertexIndex[0], &vertexIndex[1], &vertexIndex[2] );
			if (matches != 3){
				printf("File format error. Please check the file content.\n");
				return false;
			}
			// 注意：obj文件的顶点序号从1开始，而数组序号从0开始。
			faces.push_back( vertexIndex[0] - 1 );
			faces.push_back( vertexIndex[1] - 1 );
			faces.push_back( vertexIndex[2] - 1 );
		} else {											// 其它，忽略
			char stupidBuffer[1000];
			fgets(stupidBuffer, 1000, file);
		}
	}
	return true;
}

// GLUT框架消息处理函数 ///////////////////////////////////////////////////////

// 初始化OpenGL，生成窗口后调用。
void init()
{
	if( _smooth_shading )
		glShadeModel( GL_SMOOTH );						// 使用光滑渲染
	else
		glShadeModel( GL_FLAT );						// 使用平坦渲染
	glClearColor( 0.0, 0.0, 0.0, 0.5f );				// 颜色缓存清屏值（RGBA）
	glClearDepth( 1.0 );								// 深度缓存清屏值
	glEnable( GL_DEPTH_TEST );							// 使用深度测试
	glDepthFunc( GL_LEQUAL );							// 设置深度测试方法
	glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );// 设置最佳透视投影矫正方法
	_wireframe 
		? glPolygonMode( GL_FRONT_AND_BACK, GL_LINE ) 	// 渲染模式：线框多边形
		: glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );	// 渲染模式：填充多边形
	glCullFace( GL_BACK );								// 设定去除模式：去除背向面
	_cull_back_faces 
		? glEnable( GL_CULL_FACE )						// 打开去除背向面
		: glDisable( GL_CULL_FACE );					// 关闭去除背向面

	// 设置光照和材质 /////////////////////////////////////
	glColorMaterial( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );// 设置颜色材质参数
	glEnable( GL_COLOR_MATERIAL );						// 使用颜色材质
	glColor3f( 1.0, 1.0, 1.0 );							// 设置默认的物体颜色
	glEnable( GL_LIGHTING );							// 使用光照

	// 添加0号灯
	GLfloat LightAmbient[]  = { 0.2, 0.2, 0.2, 1.0 };	// 环境光颜色
	GLfloat LightDiffuse[]  = { 0.7, 0.7, 0.7, 1.0 };	// 散射光颜色
	GLfloat LightSpecular[] = { 1.0, 1.0, 1.0, 1.0 };	// 高光颜色
	GLfloat LightPosition[] = { 1.0, 1.0, 1.0, 0.0 };	// 平行光（注意最后参数是0，表示向量）
	glLightfv( GL_LIGHT0, GL_AMBIENT,  LightAmbient );	// 设置环境光
	glLightfv( GL_LIGHT0, GL_DIFFUSE,  LightDiffuse );	// 设置散射光
	glLightfv( GL_LIGHT0, GL_SPECULAR, LightSpecular );	// 设置高光
	glLightfv( GL_LIGHT0, GL_POSITION, LightPosition );	// 设置平行光
	glEnable( GL_LIGHT0 );								// 打开0号灯

	// 其它 ///////////////////////////////////////////////
	_justnow = _fps_time = clock();						// 初始化时间
}

// 初始化场景
void init_scene( const char *path ) {
	_flock.init();
	/*
	// 清除场景信息 ///////////////////////////////////////
	_verts.clear();
	_faces.clear();
	_vert_normals.clear();
	_face_normals.clear();

	// 打开模型文件 ///////////////////////////////////////
	if( !load_obj_file( path, _verts, _faces ) ) {
		printf("Error occured when initilizing the scene.");
		return;
	}

	// 计算法线 ///////////////////////////////////////////

	unsigned int i0, i1, i2;
	double x0, y0, z0, x1, y1, z1, x2, y2, z2;
	double ux, uy, uz, vx, vy, vz;
	double nx, ny, nz;
	double length;

	// 初始化顶点法线
	_vert_normals.assign( _verts.size(), 0.0 );

	// 计算面法线
	for( unsigned int i = 0; i < _faces.size() / 3; ++i ) {		// 遍历所有三角形，每3个顶点为一组
		// 取得该三角形的3个顶点的序号
		i0 = _faces[ i*3   ] * 3;
		i1 = _faces[ i*3+1 ] * 3;
		i2 = _faces[ i*3+2 ] * 3;

		// 取得该三角形的3个顶点的坐标
		x0 = _verts[ i0 ];   y0 = _verts[ i0+1 ];   z0 = _verts[ i0+2 ];
		x1 = _verts[ i1 ];   y1 = _verts[ i1+1 ];   z1 = _verts[ i1+2 ];
		x2 = _verts[ i2 ];   y2 = _verts[ i2+1 ];   z2 = _verts[ i2+2 ];

		// 计算两条边向量
		ux = x1 - x0;	uy = y1 - y0;	uz = z1 - z0;
		vx = x2 - x0;	vy = y2 - y0;	vz = z2 - z0;
		
		// 计算叉积
		nx = uy * vz - uz * vy;		ny = uz * vx - ux * vz;		nz = ux * vy - uy * vx;

		// 长度正规化，将面法线缩放为单位长度
		length = sqrt( nx*nx + ny*ny + nz*nz );						// 计算长度
		if( length < 0.00001f )
			length = 0.00001f;										// 要作除数，以防为零
		length = 1.0 / length;										// 预先计算倒数
		nx = nx * length;	ny = ny * length;	nz = nz * length;	// 长度缩放为1

		// 保存面法线
		_face_normals.push_back( nx );	_face_normals.push_back( ny );	_face_normals.push_back( nz );

		// 将面法线累加到顶点法线，权重为1
		_vert_normals[ i0 ] += nx;   _vert_normals[ i0+1 ] += ny;   _vert_normals[ i0+2 ] += nz;	// 累加到第1个顶点法线
		_vert_normals[ i1 ] += nx;   _vert_normals[ i1+1 ] += ny;   _vert_normals[ i1+2 ] += nz;	// 累加到第2个顶点法线
		_vert_normals[ i2 ] += nx;   _vert_normals[ i2+1 ] += ny;   _vert_normals[ i2+2 ] += nz;	// 累加到第3个顶点法线
	}

	// 顶点法线长度正规化，将顶点法线缩放为单位长度
	for( unsigned int i = 0; i < _vert_normals.size() / 3; ++i ) {
		// 取得累加结果
		nx = _vert_normals[ i*3   ];
		ny = _vert_normals[ i*3+1 ];
		nz = _vert_normals[ i*3+2 ];

		// 长度正规化
		length = sqrt( nx*nx + ny*ny + nz*nz );						// 计算长度
		if( length < 0.00001f )
			length = 0.00001f;										// 要作除数，以防为零
		length = 1.0 / length;										// 预先计算倒数
		_vert_normals[ i*3   ] = nx * length;						// 长度缩放为1
		_vert_normals[ i*3+1 ] = ny * length;
		_vert_normals[ i*3+2 ] = nz * length;
	}
	*/
}

// 每当窗口缩放调用；init()后调用一次。
// w：以像素计算的窗口宽度
// h：以像素计算的窗口高度
void reshape( int w, int h )
{
	// 因h要做除数计算投影纵横比例，须预防其为零
	if( h == 0 )
		h = 1;
	
	// 设置viewport
	glViewport(0, 0, w, h);			// 视区（viewport）占用整个窗口

	// 设置Projection矩阵
	glMatrixMode( GL_PROJECTION );	// 选择设定投影（Projection）矩阵
	glLoadIdentity();				// 将之重设为单位矩阵
	gluPerspective(45.0,			// 纵向视角
		1.0 * w / h,				// 画面纵横比
		0.1,						// 近截平面
		400.0);						// 远截平面

	// 设置Model-View矩阵M
	glMatrixMode( GL_MODELVIEW );	// 选择设定Model-View矩阵
	glLoadIdentity();				// 将之重设为单位矩阵，M=I
}

// 每当需要重绘时调用；init()和reshape()后调用。
void display( void )
{
	// 计时 ///////////////////////////////////////////////
	_now = clock();					// 当前时钟周期
	double sec = ((double)(_now - _justnow)) / CLOCKS_PER_SEC;	// 流逝秒
	_justnow = _now;				// 保存当前时钟周期

	// 计算帧速 ///////////////////////////////////////////
	_fps_cnt += 1.0;				// 帧数+1
	// 每 _fps_interval 秒输出一次帧速
	if( _now - _fps_time > CLK_TCK * _fps_interval )		// CLK_TCK=1000
	{
		_fps_time = _now;
		sprintf( _title, "Mini 3D Engine - %dfps", _fps_cnt / _fps_interval );
		glutSetWindowTitle( _title );
		_fps_cnt = 0;
	}

	// 清屏（颜色缓存、深度缓存） /////////////////////////
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	// 设置Model-View矩阵M ////////////////////////////////
	glLoadIdentity();				// 将之重设为单位矩阵，M=I
	gluLookAt(						// 设置摄像机，使M=C
		_flock._cam.x(), _flock._cam.y(), _flock._cam.z(),				// 摄像机的位置
		_flock._center.x(), _flock._center.y(), _flock._center.z(),		// 观察目标的位置
		0.0, 1.0, 0.0);													// 摄像机的“上”方

	// 绘制场景 ///////////////////////////////////////////

	if( _rot_x > 90.0 )				// 限制旋转值以获得良好的观察效果
		_rot_x = 90.0;
	else if( _rot_x < -90.0 )
		_rot_x = -90.0;

	glPushMatrix();					// 当前Model-View矩阵（M=C）入栈，下面以世界坐标系绘制场景

	glTranslated( 0.0, 0.0, 0.0 );			// 修改Model-View矩阵，使M=C*T
	glRotated( _rot_x, 1.0, 0.0, 0.0 );		// 修改Model-View矩阵，使M=C*T*Rx
	glRotated( _rot_y, 0.0, 1.0, 0.0 );		// 修改Model-View矩阵，使M=C*T*Rx*Ry
	
	// 绘制背景
	glPushMatrix();
	glRotated( 90.0, 1.0, 0.0, 0.0 );
	glColor3f( 0.5, 0.5, 1.0 );
	glutWireSphere( _flock._scene_radius * 2, 100, 50 );
	glPopMatrix();

	//glScaled( _scale, _scale, _scale );		// 修改Model-View矩阵，使M=C*T*Rx*Ry*S
	
	_flock.update( sec );			// 计算场景变化
	_flock.render();				// 渲染场景
	
	glPopMatrix();					// Model-View矩阵M出栈，恢复M=C

	// 切换缓冲区 /////////////////////////////////////////
	glutSwapBuffers();
}

// 键盘普通字符消息处理函数
void keyboard ( unsigned char key, int x, int y )
{
	key = tolower( key );

	_flock.keyboard_char( key, x, y );

	switch( key )
	{
	case 27:						// ESC: 退出程序
		std::exit( 0 );
		break;

	case 'z':						// z: 物体放大
		_scale += 0.05f;
		break;

	case 'x':						// x: 物体缩小
		_scale -= 0.05f;
		break;

	case 'w':						// w: 摄像机平移
		_camy -= 0.05f;
		_tgty -= 0.05f;
		break;

	case 's':						// s: 摄像机平移
		_camy += 0.05f;
		_tgty += 0.05f;
		break;

	case 'a':						// a: 摄像机平移
		_camx += 0.05f;
		_tgtx += 0.05f;
		break;

	case 'd':						// d: 摄像机平移
		_camx -= 0.05f;
		_tgtx -= 0.05f;
		break;

	case 'q':						// q: 摄像机平移
		_camz -= 0.05f;
		_tgtz -= 0.05f;
		break;

	case 'e':						// e: 摄像机平移
		_camz += 0.05f;
		_tgtz += 0.05f;
		break;
	
	case 'r':						// r: 摄像机复位
		_camx = _camy = 0.0;
		_camz = 3.0;
		_tgtx = _tgty = _tgtz = 0.0;
		break;

	case 'h':						// h: 帮助信息
		printf( _help );
		break;
	
	default:
		break;
	}
}

// 键盘特殊字符消息处理函数
void arrow_keys( int a_keys, int x, int y )
{
	switch( a_keys )
	{
	case GLUT_KEY_UP:				// ARROW UP
		_rot_x += 1.0;
		break;

	case GLUT_KEY_DOWN:				// ARROW DOWN
		_rot_x -= 1.0;
		break;

	case GLUT_KEY_RIGHT:			// ARROW RIGHT
		_rot_y += 1.0;
		break;

	case GLUT_KEY_LEFT:				// ARROW LEFT
		_rot_y -= 1.0;
		break;

	case GLUT_KEY_F1:				// F1: 全屏显示
		_fullscreen = !_fullscreen;
		printf("Full Screen: %s\n", "Off\0On" + 4 * _fullscreen);
		if( _fullscreen )
			glutFullScreen();				// 全屏显示
		else 
		{
			glutReshapeWindow( 800, 600 );	// 窗口像素尺寸
			glutPositionWindow( 50, 50 );	// 窗口位置
		}
		break;

	case GLUT_KEY_F2:				// F2: 线框渲染/面渲染
		_wireframe = !_wireframe;
		printf("Wireframe: %s\n", "Off\0On" + 4 * _wireframe);
		_wireframe ? glPolygonMode( GL_FRONT_AND_BACK, GL_LINE ) : glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
		break;

	case GLUT_KEY_F3:				// F3: 光滑渲染/平坦渲染
		_smooth_shading = !_smooth_shading;
		printf("Smooth Shading: %s\n", "Off\0On" + 4 * _smooth_shading);
		_smooth_shading ? glShadeModel( GL_SMOOTH ) : glShadeModel( GL_FLAT );
		break;

	case GLUT_KEY_F4:				// F4: 剔除背向面
		_cull_back_faces = !_cull_back_faces;
		printf("Cull Back Faces: %s\n", "Off\0On" + 4 * _cull_back_faces);
		_cull_back_faces ? glEnable( GL_CULL_FACE ) : glDisable( GL_CULL_FACE );
		break;

	case GLUT_KEY_F5:				// F5: 打开/关闭光照
		_lighting = !_lighting;
		printf("Lighting: %s\n", "Off\0On" + 4 * _lighting);
		_lighting ? glEnable( GL_LIGHTING ) : glDisable( GL_LIGHTING );
		break;

	default:
		break;
	}
}

// 鼠标按键消息处理函数
void mouse_buttons(int button, int state, int x, int y)
{
	switch( button )
	{
	case  GLUT_LEFT_BUTTON:
		if( state == GLUT_DOWN )		// LEFT BUTTON DOWN
		{
			_mouse_state = GLUT_DOWN;
			_mouse_button = button;
			_mouse_x = x;
			_mouse_y = y;
		}
		else							// LEFT BUTTON UP
			_mouse_state = GLUT_UP;
		break;

	case  GLUT_MIDDLE_BUTTON:
		break;

	case  GLUT_RIGHT_BUTTON:
		break;

	default:
		break;
	}
}

// 鼠标移动消息处理函数
void mouse_moving( int x, int y )
{
	if( _mouse_state == GLUT_DOWN )
	{
		int dx, dy;
		switch( _mouse_button )
		{
		case  GLUT_LEFT_BUTTON:
			// 计算鼠标移动量
			dx = x - _mouse_x;
			dy = y - _mouse_y;

			// 修改摄像机或场景
			_rot_x += dy;
			_rot_y += dx;

			// 保存鼠标当前位置，留作后用
			_mouse_x = x;
			_mouse_y = y;
			break;

		case  GLUT_MIDDLE_BUTTON:
			break;

		case  GLUT_RIGHT_BUTTON:
			break;

		default:
			break;
		}
	}
}

// 主函数
// argc：参数数目
// argv：参数字符串
int main( int argc, char** argv )
{
	// 初始化GLUT
	glutInit( &argc, argv );				// 初始化GLUT
	glutInitDisplayMode( GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA );	// 显示模式：深度测试、双缓冲、RGBA颜色模型
	glutInitWindowPosition( 50, 50);		// 窗口位置
	glutInitWindowSize( 800, 600 );			// 窗口像素尺寸
	glutCreateWindow( "Mini 3D Engine" );	// 窗口标题
	init();									// 初始化
	init_scene( argv[1] );
	
	if( _fullscreen )
		glutFullScreen();					// 全屏显示

	// 设置各类消息处理函数
	glutDisplayFunc( display );				// 窗口绘制处理函数
	glutReshapeFunc( reshape );				// 窗口缩放处理函数
	glutKeyboardFunc( keyboard );			// 键盘普通字符消息处理函数
	glutSpecialFunc( arrow_keys );			// 键盘特殊字符消息处理函数
	glutMouseFunc( mouse_buttons );			// 鼠标按键消息处理函数
	glutMotionFunc( mouse_moving );			// 鼠标移动消息处理函数
	glutIdleFunc( display );				// 空闲处理函数
	
	// 打印帮助信息
	printf( _help );

	// 进入消息循环，直至关闭窗口
	glutMainLoop();
}



