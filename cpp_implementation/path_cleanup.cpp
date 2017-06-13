/**
*  This file is supposed to be path_cleanup.py, reimplemented in C++. Don't bother trying to compile it, it won't work.
*  I just needed a place to dump the random methods.
*/

#define SMALL_FLOAT  0.0000001

#define POSITION_DELTA 2 //how many meters to move before appending a new position to return_path
#define PRUNING_DELTA (POSITION_DELTA * 1.5) //how many meteres apart must two points be, such that we can assume that there is no obstacle between those points
#define RDP_EPSILON (POSITION_DELTA * 0.5)
#define MAX_PATH_LEN 100 //The amount of memory used by safe RTL will be slightly higher than 3*8*MAX_PATH_LEN bytes. Increasing this number will improve path pruning, but will use more memory, and running a path cleanup will take longer.

// dot product is already defined using the * operator with Vector3f objects
// cross product is %, length() also exists

#define HYPOT(a,b) (a-b).length()


struct dist_point
{
  float distance;
  Vector3f point;
};
dist_point d_p = {1, 2.0};
Vector3f(1.0f, 2.0f, 3.0f)

/**
*  Returns the closest distance in 3D space between any part of two input segments, defined from p1 to p2 and from p3 to p4.
*  Also returns the point which is halfway between
*
*  Limitation: This function does not work for parallel lines. In this case, it will return FLT_MAX. This does not matter for the path cleanup algorithm because
*  the pruning will still occur fine between the first parallel segment and a segment which is directly before or after the second segment.
*/
dist_point segment_segment_dist(Vector3f p1, Vector3f p2, Vector3f p3, Vector3f p4){
  Vector3f u = p2-p1;
  Vector3f v = p4-p3;
  Vector3f w = p1-p3;

  float a = u*u;
  float b = u*v;
  float c = v*v;
  float d = u*w;
  float e = v*w;

  // the parameter for the position on line1 and line2 which define the closest points.
  float t1 = 0f;
  float t2 = 0f;

  if ( (a*c)-(b*b) < SMALL_FLOAT){ // almost parallel. This avoids division by 0.
    return FLT_MAX;
  } else{
    t1 = (b*e-c*d)/(a*c-b*b);
    t2 = (a*e-b*d)/(a*c-b*b);

    // restrict both parameters between 0 and 1.
    t1 = min(max(0,t1),1);
    t2 = min(max(0,t2),1);

    // difference between two closest points
    Vector3f dP = w+t1*u-t2*v;

    halfway_point = (p1+t1*u + p3+t2*v)/2.
    return {dP.length(), halfway_point}
  }
}
