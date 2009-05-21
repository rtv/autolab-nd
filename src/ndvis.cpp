
/**
 * Stage visualization for ND
 * Author: Richard Vaughan 2009
 * $Id$
 */

#include "ndvis.h"

static void DrawCircle( float x, float y, float z, float radius, float steps )
{
  glBegin(GL_LINE_LOOP);
  for( float a = 0;  a < 2.0*M_PI; a+= 2.0*M_PI/steps )
	 glVertex3f( x + radius * sin(a),
					 y + radius * cos(a),
					 z );
  glEnd();
}

void NdVis::Visualize ( Stg::Model* mod, Stg::Camera* cam ) 
{
  Stg::Geom geom = mod->GetGeom();

  glPushMatrix();
		
  Stg::Gl::pose_inverse_shift( mod->GetGlobalPose() );

  if( nd.hasActiveGoal() )
	 {
		CPose2d goal = nd.getGoal();
		glPointSize( 10 );
		mod->PushColor( 1,0,0,0.8 ); // red
		glBegin( GL_POINTS );
		glVertex2f( goal.mX, goal.mY );
		glEnd();
		char buf[64];
		snprintf( buf, 64, "Goal (%.2f,%.2f)", goal.mX, goal.mY );
		Stg::Gl::draw_string( goal.mX + 0.2, goal.mY + 0.2, 0, buf );
		mod->PopColor();
	 }
		
  for ( int i = 0; i < nd.mObstacles.longitud; i++ ) 
	 {
		glPointSize( 5 );
		mod->PushColor( 1,0,0,0.8 ); // red
		glBegin( GL_POINTS );
		glVertex2f( nd.mObstacles.punto[i].x,
						nd.mObstacles.punto[i].y );
		glEnd();
		mod->PopColor();
	 }				

  glPopMatrix(); // back to robot coords
		
  mod->PushColor( 0,0,1,0.8 ); // blue
  float dx =  1.0 * cos( nd.mInfo.angulo );
  float dy =  1.0 * sin( nd.mInfo.angulo );
  glBegin( GL_LINES );		
  glVertex2f( 0, 0 );
  glVertex2f( dx, dy );		
  glEnd();
  mod->PopColor();

  mod->PushColor( 0,1,0,1 ); // green
  DrawCircle( 0,0,0, nd.mSafetyDist, 20 );
  mod->PopColor();
		
  glPointSize( 5 );
  mod->PushColor( 1,0,0,0.8 ); // red
  for ( unsigned int i = 0; i < SECTORES; i++ ) 
	 {
		if(  nd.mInfo.d[i].r < 0 )
		  continue;

		glBegin( GL_LINES );
			 
		float dx = nd.mInfo.d[i].r *  cos( nd.mInfo.d[i].a );
		float dy = nd.mInfo.d[i].r *  sin( nd.mInfo.d[i].a );

		glVertex2f( 0, 0 );
		glVertex2f( dx, dy );

		glEnd();
	 }				
  mod->PopColor();
}