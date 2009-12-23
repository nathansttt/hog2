#include "astar.h"

template<>
void SFBDSAStar<graphState,graphMove>::OpenGLDraw( GLdouble rad ) const {

	// draw closed list in green
	glColor3f(0., 1., 0.);
	for( AStarClosedList::iterator closedit = closed.begin();
	     closedit != closed.end(); closedit++ ) {
		GLdouble x,y,z;

		// draw start state
		node* n = ((GraphEnvironment*)env)->GetGraph()->GetNode( closedit->s1 );
		x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
		y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
		z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
		DrawSphere( x, y, z, rad );

		// draw end state
		n = ((GraphEnvironment*)env)->GetGraph()->GetNode( closedit->s2 );
		x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
		y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
		z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
		DrawSphere( x, y, z, rad );
	}

	return;
};

template<>
void SFBDSAStar<graphState,graphMove>::OpenGLDraw( QueueNode q, bool side, GLdouble rad ) const {

	GLdouble x,y,z;

	if( side )
		glColor3f(1.,0.,0.);
	else
		glColor3f(0.,0.,1.);
	// draw the start
	node *n = ((GraphEnvironment*)env)->GetGraph()->GetNode( q.s1 );
	x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
	y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
	z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
	DrawSphere( x, y, z, rad );

	if( side )
		glColor3f(0.,0.,1.);
	else
		glColor3f(1.,0.,0.);
	n = ((GraphEnvironment*)env)->GetGraph()->GetNode( q.s2 );
	x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
	y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
	z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
	DrawSphere( x, y, z, rad );

	return;
};


template<>
void SFBDSAStar<xyLoc,tDirection>::OpenGLDraw( GLdouble rad ) const {
	// todo
};

template<>
void SFBDSAStar<xyLoc,tDirection>::OpenGLDraw( QueueNode q, bool side, GLdouble rad ) const {
	// todo
};

