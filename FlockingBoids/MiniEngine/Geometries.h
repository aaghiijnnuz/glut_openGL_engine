#pragma once

#include <gl\glut.h>				// GLUT
#include <gl\gl.h>				// GL
#include <gl\glu.h>
//#include <gl\glaux.h>

void draw_helicopter( double scale ) {
	glPushMatrix();
	glScaled( 0.75f * scale, 0.75f * scale, 0.75f * scale );

	glPushMatrix();
	glRotated( 90.0f, 0.0f, 1.0f, 0.0f );
	glScaled( 1.0f, 1.0f, 1.3f );
	glutSolidSphere( 0.25f, 5, 5 );			// ª˙…Ì
	glPopMatrix();

	/*glPushMatrix();
	glTranslated( 0.0f, -0.21f, 0.0f );
	glScaled( 3, 1, 10 );
	glutSolidCube( 0.04 );
	glPopMatrix();*/

	glPushMatrix();
	glTranslated( 0.0f, -0.24f, 0.0f );
	glRotated( 90.0f, 0.0f, 1.0f, 0.0f );
	glScaled( 1.0f, 1.0f, 5.0f );
	glTranslated( 0.2f, 0.0f, 0.0f );
	glutSolidSphere( 0.07f, 3, 3 );			// Ω≈º‹
	glTranslated( -0.4f, 0.0f, 0.0f );
	glutSolidSphere( 0.07f, 3, 3 );
	glPopMatrix();

	glPushMatrix();
	glTranslated( 0.0f, 0.2f, 0.0f );
	glRotated( -90.0f, 1.0f, 0.0f, 0.0f );
	glutSolidCone( 0.04, 0.16, 4, 1 );		// –˝“Ì÷·
	glPopMatrix();

	glPushMatrix();
	glTranslated( 0.0f, 0.4f, 0.0f );
	glRotated( 90.0f, 1.0f, 0.0f, 0.0f );
	glutSolidCone( 0.4, 0.05, 16, 1 );		// –˝“Ì
	glPopMatrix();

	glPushMatrix();
	glRotated( -90.0f, 0.0f, 1.0f, 0.0f );
	glTranslated( 0.0f, 0.05f, 0.29f );
	glRotated( -15.0f, 1.0f, 0.0f, 0.0f );
	glutSolidCone( 0.05, 0.3, 4, 1 );		// ª˙Œ≤

	glTranslated( 0.03f, 0.0f, 0.3f );
	glRotated( 90.0f, 0.0f, 1.0f, 0.0f );
	glutSolidCone( 0.15, 0.0, 8, 1 );		// Œ≤–˝“Ì
	glPopMatrix();

	glPopMatrix();
}

void draw_plane( double scale ) {
	glPushMatrix();
	glScaled( 0.75f * scale, 0.75f * scale, 0.75f * scale );

	glPushMatrix();
	glRotated( 90.0f, 0.0f, 1.0f, 0.0f );
	glScaled( 0.7f, 1.0f, 2.5f );
	glutSolidSphere( 0.5f / 3.0f, 5, 5 );			// ª˙…Ì
	glPopMatrix();

	glPushMatrix();
	glTranslated( 0.05f, -0.09f, 0.0f );
	glScaled( 1.0f, 0.2f, 3.0f );
	glutSolidSphere( 0.5f / 3.0f, 3, 5 );			// ª˙“Ì
	glPopMatrix();

	glPushMatrix();
	glTranslated( -0.3f, 0.0f, 0.0f );
	glScaled( 1.0f, 0.2f, 2.0f );
	glutSolidSphere( 0.1f, 3, 5 );					// ∆ΩŒ≤
	glPopMatrix();

	glPushMatrix();
	glTranslated( -0.3f, 0.15f, 0.0f );
	glRotated( -90.0f, 1.0f, 0.0f, 0.0f );
	glRotated( -10.0f, 0.0f, 1.0f, 0.0f );
	glScaled( 1.0f, 0.15f, 2.0f );
	glutSolidSphere( 0.1f, 4, 5 );					// ¥πŒ≤
	glPopMatrix();

	glPopMatrix();
}


