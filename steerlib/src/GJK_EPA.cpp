#include "obstacles/GJK_EPA.h"
#include <algorithm>
#include <vector>
#include <math.h>

SteerLib::GJK_EPA::GJK_EPA() {}

Util::Vector tripleProduct( Util::Vector& a, Util::Vector& b, Util::Vector& c );
Util::Vector furthestPoint( std::vector<Util::Vector> shape, Util::Vector d ); 
Util::Vector support ( std::vector<Util::Vector> shapeA, std::vector<Util::Vector> shapeB, Util::Vector d ); 
std::vector<Util::Vector> minkowski_difference( const std::vector<Util::Vector> shapeA, const std::vector<Util::Vector> shapeB );
bool containsOrigin( std::vector<Util::Vector>&simplex, Util::Vector&direction );
bool GJK( std::vector<Util::Vector> shapeA, std::vector<Util::Vector> shapeB, std::vector<Util::Vector>&simplex ); 
void EPA( std::vector<Util::Vector> shapeA, std::vector<Util::Vector> shapeB, std::vector<Util::Vector>& simplex, Util::Vector& penetration_vector, float& penetration_depth );
void findClosestEdge ( std::vector<Util::Vector>& simplex, double& distance, int& index, Util::Vector& normal ); 

/*******************************************************************
* Computes the tripple product of three vectors.
*******************************************************************/

Util::Vector tripleProduct( 
	Util::Vector& a, 
	Util::Vector& b, 
	Util::Vector& c )
{
	return b*dot( a,c ) - c*dot( a,b );
}

/*******************************************************************
*Finds the minkowski difference between two shapes.
*******************************************************************/

std::vector<Util::Vector> minkowski_difference(
	const std::vector<Util::Vector> shapeA, 
	const std::vector<Util::Vector> shapeB )
{

	std::vector<Util::Vector> difference;
	for(Util::Vector a : shapeA) {
		for(Util::Vector b : shapeB) {
			Util::Vector dif;
			dif.x = a.x - b.x;
			dif.z = a.z - b.z;
			difference.insert(difference.begin(), dif);
		}
	}
	difference.erase(unique(difference.begin(), difference.end(), 
	[](const Util::Vector dif1, const Util::Vector dif2) -> bool {
		return dif1.x == dif2.x && dif1.z == dif2.z;
	}), difference.end());
	return difference;
}

/*******************************************************************
* Finds the point of the shape that has been input that is the 
* futherest point of the shape with the given direction.
*******************************************************************/

Util::Vector furthestPoint(
	std::vector<Util::Vector> shape, 
	Util::Vector d ) 
{
	float highest = -FLT_MAX;
	int index;
	int len = shape.size();

	for ( int i = 0;  i< len; ++i ) {
		float dp = dot(shape[i], d);
		if ( highest < dp ) {
			highest = dp;
			index = i;
		}
	}
	//printf("furthestPoint: (%f, %f, %f)\n", shape[index].x, shape[index].y, shape[index].z);
	return shape[index];
}

/*******************************************************************
*Finds the minkowski difference between two shapes.
*******************************************************************/

Util::Vector support ( 
	std::vector<Util::Vector> shapeA, 
	std::vector<Util::Vector> shapeB, 
	Util::Vector d ) 
{
	using namespace std;
	using namespace Util;

	Vector p1 = furthestPoint(shapeA, d);
	Vector d2 = d*(-1);
	Vector p2 = furthestPoint(shapeB, d2);
	return Util::Vector (p1.x-p2.x, p1.y-p2.y, p1.z - p2.z);
}

/*******************************************************************
*GJK is used to determine if a collision has occured, and if one
*does it create a simplex for the collision.
*******************************************************************/

bool GJK( 
	std::vector<Util::Vector> shapeA, 
    std::vector<Util::Vector> shapeB,  
    std::vector<Util::Vector>&simplex ) 
{
	using namespace Util;
	using namespace std;	
	
	Vector d = {0,0,1};
	vector<Vector> m_diff = minkowski_difference(shapeA, shapeB);
	Vector p = support(shapeA, shapeB, d);
	simplex.push_back(p);

	d = d*(-1);
	//printSimplex(simplex);
	while (true)
	{

		p = support(shapeA, shapeB, d);
		simplex.push_back(p);
		
		if ( dot(simplex.back(), d) < 0 ) {
			//printf("point 1\n");
			return false;
		}else{
			//printf("point 2\n");
			if ( containsOrigin(simplex,d) ){
				//printf("point 3\n");
				return true;
			} 
		}
		
	}
	return false;
}

/*******************************************************************
*Contains Origin checks to see if the simplex contains the origin.
*If it does it will find the simplex that is closest to the origin,
*and also determines the direction of the simplex.
*******************************************************************/

bool containsOrigin(
	std::vector<Util::Vector>&simplex, 
	Util::Vector&direction )
{

	using namespace std;
	using namespace Util;

	Vector a = simplex.back();
	Vector neg_a = -1*a;
	Vector b, c, ac, ab, acPerp, abPerp;

	if(simplex.size() == 3){
		
		b = simplex[1];
		c = simplex[0];
		ab = b - a;
		ac = c - a;
		abPerp = tripleProduct( ac, ab, ab );
		acPerp = tripleProduct( ab, ac, ac );
		
		if (dot(abPerp,neg_a) > 0 ) {
			simplex.erase(simplex.begin());
			direction= abPerp;
		
		} else {
			if (dot( acPerp, neg_a ) > 0 ) {
				simplex.erase(simplex.end()-1);
				direction = acPerp;
			
			} else {
				return true;
			}
		}
	} else {
			b = simplex[0];
			ab = b - a;
			abPerp = tripleProduct(ab, neg_a, ab);
			direction = abPerp;	
	}

	return false;
}

/*******************************************************************
*Use to determine the penetration depth and penetration vector 
*between two shapes after a collision has been detected
*******************************************************************/

void EPA( 
	std::vector<Util::Vector> shapeA, 
 	std::vector<Util::Vector> shapeB, 
 	std::vector<Util::Vector>& simplex, 
 	Util::Vector& penetration_vector,
 	float& penetration_depth )
{
	
	using namespace Util;
	using namespace std;

	int index;
	double tolerance, distance, d;
	Vector p, normal;
	
	tolerance = 0.001;
	
	while(true)
	{

		findClosestEdge(simplex, distance, index, normal );
		
		p = support(shapeA, shapeB, normal);
		d = dot(p,normal);
	
		if(d - distance < tolerance){
			penetration_vector = normal;
			penetration_depth = d;
			return;
		} else {
			simplex.insert(simplex.begin()+index, p);
		}
	}
}

/*******************************************************************
* Finds the distance, normal, and index of the closest edge of a 
* simplex.
*******************************************************************/

void findClosestEdge ( 
	std::vector<Util::Vector>& simplex, 
	double& distance, 
	int& index,
	Util::Vector& normal ) 
{
	using namespace Util;
	using namespace std;

	Vector edge, a, b, e, origA , n;
	double d;
	int i, j;
	distance = 10000000.0;


	for ( i = 0; i < simplex.size(); i++ ) 
	{
		
		j = i+1 == simplex.size() ? 0 : i+1;
		a = simplex[i];
		b = simplex[j];
		e = b-a;
		origA = a;
		n = normalize(tripleProduct(e,origA,e));
		d = dot(n,a);

		if ( d < distance ) {
			distance = d;
			normal = n;
			index = j;
		}
	}
}


bool SteerLib::GJK_EPA::intersect( 
	float& return_penetration_depth, 
	Util::Vector& return_penetration_vector, 
	const std::vector<Util::Vector>& _shapeA, 
	const std::vector<Util::Vector>& _shapeB )
{
	std::vector<Util::Vector> simplex;
	//float penetration_depth;
	Util::Vector penetration_vector;
	if ( GJK( _shapeA, _shapeB, simplex ) ){
	// if there is an collision
		EPA(_shapeA, _shapeB, simplex, return_penetration_vector, return_penetration_depth );
		return true;
	}else{
		return false;
	}
}
