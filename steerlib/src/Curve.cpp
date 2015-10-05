//
// Copyright (c) 2015 Mahyar Khayatkhoei
// Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)

{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI

	/*================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function drawCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================*/

	// Robustness: make sure there is at least two control point: start and end points

	if (!checkRobust()) {
		std::cerr << "ERROR>>>> drawCurve has failed on checkRobust!!!" <<std::endl;
		return;
	}

	int i;
	int intervalSize;
	float timeInterval;
	float currentTime;
	Point currentPoint;
	Point nextPoint;	

	// Using the window as a step size, we split up the intervals that we want to interpolate to form the curve
	timeInterval = controlPoints[controlPoints.size() - 1].time - controlPoints[0].time;
	intervalSize = (int) (timeInterval / window);

	currentPoint = controlPoints[0].position;
	currentTime = controlPoints[0].time + window;

	for (i = 0; i < intervalSize; i++) {
		if (i == (intervalSize - 1)) { // this would be the second to last interval or the last drawn portion
			nextPoint = controlPoints[controlPoints.size() - 1].position;
			currentTime = controlPoints[controlPoints.size() - 1].time;
			DrawLib::drawLine(currentPoint, nextPoint, curveColor, curveThickness);
			return;
		}
		if (calculatePoint(nextPoint, currentTime) == false) {
			std::cerr << "ERROR>>>> drawCurve has failed on calculatePoint!!!" <<std::endl;
			return;
		}
		
		currentTime += window;

		DrawLib::drawLine(currentPoint, nextPoint, curveColor, curveThickness);

		currentPoint = nextPoint;
	}
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points


	/*****************************
BROKEN NEEDS LINEAR INTERPOLATION
	int i;
	float timeIn;
	float timeOut;
	float intervalTime;
	float deltaTime;
	float currTime;
	Point inPoint;
	Point outPoint;
	for (i = 1; i < controlPoints.size(); i++) {
		timeIn = controlPoints[i-1].time;
		timeOut = controlPoints[i].time;
		intervalTime = timeOut - timeIn;
		deltaTime = intervalTime/(float)window;
		inPoint = controlPoints[i].position;
		
			
			if ( type == hermiteCurve ) {
				outPoint = useHermiteCurve(i, currTime);
			} else if (type == catmullCurve) {
				outPoint = useCatmullCurve(i, currTime);
			}
			
			DrawLib::drawLine(inPoint,outPoint,curveColor,curveThickness);
	}
	*****************************/	
	return;
#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	int i;
	int j;

	for (i = 1 ; i < controlPoints.size() ; i++){
		CurvePoint temp = controlPoints[i];


		for ( j = i-1 ; j >= 0 && temp.time < controlPoints[j].time ; j-- ) {
			controlPoints[j+1]=controlPoints[j];
		}

		controlPoints[j+1] = temp;
	}



	controlPoints.erase(unique(controlPoints.begin(), controlPoints.end(), 
		[](const CurvePoint p1, const CurvePoint p2) -> bool {
			return p1.time == p2.time;
		}
	), controlPoints.end());

	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;
	float normalTime, intervalTime;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	int size = getControPoints().size();

	if ( size >= 2 ) {
		return true;
	} else {
		return false;
	}
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	int i;

	for ( i = 0 ; i < getControPoints().size() ; i++ ) {
		if ( time < getControPoints()[i].time ) {
			if (i == 0) {
				return false;
			}
			nextPoint = i;
			return true;
		}
	}

	return false;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;


	const unsigned int prevPoint = nextPoint -1; 

	// Calculate time interval, and normal time required for later curve calculations
	intervalTime = controlPoints[nextPoint].time - controlPoints[nextPoint-1].time;
	normalTime = (time - controlPoints[nextPoint-1].time)/intervalTime;

	// Calculate position at t = time on Hermite curve
	float t2 = normalTime * normalTime;
	float t3   = normalTime * t2;	

	newPosition = ( 2 * t3 - 3 * t2 + 1 		 ) * getControPoints()[prevPoint].position +
			    + (    	t3 - 2 * t2 + normalTime ) * getControPoints()[prevPoint].tangent * intervalTime
		      	+ (-2 * t3 + 3 * t2     		 ) * getControPoints()[nextPoint].position
		      	+ (     t3 -     t2           	 ) * getControPoints()[nextPoint].tangent * intervalTime;

	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
    float normalTime, intervalTime;

	const unsigned int i0 = nextPoint - 2;
	const unsigned int i1 = nextPoint - 1;
	const unsigned int i2 = nextPoint;
	const unsigned int i3 = nextPoint + 1;
	
	Point p1 = getControPoints()[i1].position;
	Point p2 = getControPoints()[i2].position;

	Vector s1, s2;

	// Calculate time interval, and normal time required for later curve calculations
	intervalTime = controlPoints[i2].time - controlPoints[i1].time;
	normalTime = (time - controlPoints[i1].time)/intervalTime;

	float t2 = normalTime * normalTime;
	float t3 = normalTime * t2;	

	// Calculate position at t = time on Catmull-Rom curve
	
	if (i1 < 1) {
		s1 = 2*(p2 - p1) - (getControPoints()[i3].position - p1)/2;
		s2 = (getControPoints()[i3].position - p1) / 2;
	} else if (i3 >= getControPoints().size()) {
		s1 = (p2 - getControPoints()[i0].position) / 2;
		s2 = 2*(p2 - p1) - (p2 - getControPoints()[i0].position)/2;
	}else {
		s1 = (p2 - getControPoints()[i0].position) / 2;
		s2 = (getControPoints()[i3].position - p1) / 2;
	}
	
	newPosition = ( 2 * t3 - 3 * t2 + 1 		 ) * p1
			    + (    	t3 - 2 * t2 + normalTime ) * s1
		      	+ (-2 * t3 + 3 * t2     		 ) * p2
		      	+ (     t3 -     t2           	 ) * s2;
	
	// Return result
	return newPosition;
}
