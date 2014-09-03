
#ifndef _PI
#define _PI

const float Pi = float(std::acos(-1.0));

const float RadiansToDegrees = float(180.0/Pi);
const float DegreesToRadians = float(Pi/180);

#define RADIAN(X)	((X)*DegreesToRadians)

static float GetAngle(float cosinus, float sinus)
{	
	int sign = (sinus > 0) ? 1 : -1;
	return sign * acos(cosinus);
}

#endif // _PI
