/**
*  This file is supposed to be path_cleanup.py, reimplemented in C++. Don't bother trying to compile it, it won't work.
*  I just needed a place to dump the random methods.
*/

#include <bitset>

#define SMALL_FLOAT  0.0000001

#define POSITION_DELTA 2f // how many meters to move before appending a new position to return_path
#define PRUNING_DELTA (POSITION_DELTA * 1.5) //h ow many meteres apart must two points be, such that we can assume that there is no obstacle between those points
#define RDP_EPSILON (POSITION_DELTA * 0.5)
#define MAX_PATH_LEN 100 // the amount of memory used by safe RTL will be slightly higher than 3*8*MAX_PATH_LEN bytes. Increasing this number will improve path pruning, but will use more memory, and running a path cleanup will take longer. No longer than 255.
#define RDP_STACK_LEN 64 // the amount of memory to be allocated for the RDP algorithm to write its to do list.
// XXX A number too small for RDP_STACK_LEN can cause a buffer overflow! The number to put here is int((s/2-1)+min(s/2, MAX_PATH_LEN-s)), where s = pow(2, floor(log(MAX_PATH_LEN)/log(2)))
// To avoid this annoying math, a good-enough estimate is ciel(MAX_PATH_LEN*2./3.)

// dot product is already defined using the * operator with Vector3f objects
// cross product is %. length() also exists

#define HYPOT(a,b) (a-b).length()

struct dist_point
{
  float distance;
  Vector3f point;
};

/**
*  Returns the closest distance in 3D space between any part of two input segments, defined from p1 to p2 and from p3 to p4.
*  Also returns the point which is halfway between
*
*  Limitation: This function does not work for parallel lines. In this case, it will return FLT_MAX. This does not matter for the path cleanup algorithm because
*  the pruning will still occur fine between the first parallel segment and a segment which is directly before or after the second segment.
*/
dist_point segment_segment_dist(Vector3f p1, Vector3f p2, Vector3f p3, Vector3f p4){ // TODO maybe just pass through pointers. Same for next method
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
    return {FLT_MAX, Vector3f(0, 0, 0)};
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

/**
*  Returns the closest distance from a point to a 3D line. The line is defined by any 2 points
*  see https://stackoverflow.com/questions/1616050/minimum-perpendicular-distance-of-a-point-to-a-line-in-3d-plane-algorithm
*/
float point_line_dist(Vector3f point, Vector3f line1, Vector3f line2){
  // triangle side lengths
  float a = HYPOT(point, line1)
  float b = HYPOT(line1, line2)
  float c = HYPOT(line2, point)

  // semiperimeter of triangle
  float s = (a+b+c)/2f

  area = sqrt(max(0f,s*(s-a)*(s-b)*(s-c))) //inner part must be constrained above 0 because a triangle where all 3 points could be on a line. float rounding could push this under 0.
  return 2*area/b
}

struct start_finish
{
  uint8_t start;
  uint8_t finish;
};

class RDP_Stack {
    start_finish stack [MAX_PATH_LEN];
    start_finish * top;
  public:
    void push(start_finish *item);
    start_finish pop();
}

RDP_Stack::RDP_Stack(){
  stack = {};
  top = &stack;
}

RDP_Stack::push(start_finish *item) {
  stack[++top] = *item;
}

RDP_Stack::pop() {
  return stack[top--];
}

RDP_Stack::empty(){
  return stack == top;
}

/**
*   Returns the number of items which were removed.
*/
int rdp(Vector3f *path, int start_index, int end_index, float epsilon){
  struct start_finish sf = {start_index, end_index};
  stack.push(sf); // TODO initialize it somewhere
  global_start = start;
  // The bitmask which represents which points to keep (1) and which to delete(0)
  std::bitset<end_index-start_index> bitmask (0x0);
  bitmask.set();
  while(!stack.empty()){
    start_finish tmp = stack.pop();
    start_index = tmp->start;
    end_index = tmp->finish;

    float max_dist = 0f;
    int index = start_index;
    for (int i = index + 1; i < end_index; i++){
      if (bitmask[i-global_start]){
          float dist = point_line_dist(path[i], path[start_index], path[end_index]);
          if(dist > dist_max){
            index = i;
            max_dist = dist;
          }
      }
    }

    if (dmax > epsilon) {
      stack.push(struct start_finish{start_index, index});
      stack.push(struct start_finish{index, last_index});
    } else {
      for (int i = startIndex + 1; i < lastIndex; ++i) {
        bitmask[i-global_start] = false;
      }
    }
    // TODO now use std::move of memset or something to shift things arount, and return the number of items deleted.
}

class Path {
    // points are stored in meters from EKF origin in NED
    Vector3f path [MAX_PATH_LEN]; // TODO would a linked list be more appropriate? Definitely would be easier to prune points out.
    int last_index, worst_length;
  public:
    void append_if_far_enough(Vector3f);
    void routine_cleanup();
    void thorough_cleanup();
  private:
    bool cleanup();
}

Path::Path() {
  path = {};
  last_index = 0;
  worst_length = 0;
}

void Path::append_if_far_enough(Vector3f p) {
  if ( HYPOT(p, path[last_index]) > position_delta ){
    path[last_index++] = p;
  }
}

void Path::routine_cleanup(){
  // We only do a routine cleanup if the memory is almost full. Cleanup deletes potentially useful points,
  // so it would be bad to clean up if we don't have to
  if ( last_index > MAX_PATH_LEN - 2 ){
    Path::cleanup();
    if ( last_index > MAX_PATH_LEN - 2 ){ // if cleanup was unsuccesful
      ;// TODO crap out, cleanly
    }
  }
}

/**
*  Run this method only when preparing to initiate the RTL procedure.
*/
void Path::thorough_cleanup(){
  while (Path::cleanup());
}

/**
*    Takes a path and runs 2 cleanup steps: pruning, then simplification.
*
*    The pruning step defines line segments from point 1 to 2, point 2 to 3, ...
*    Then it compares (almost) all line segments to see how close they got, in 3D space.
*    If they got close enough, defined by the parameter 'position_delta', all path points
*    between those two line segments are deleted, and replaced by a single point halfway between
*    where the two previous line segments were closest.
*
*    This algorithm will never compare two consecutive line segments. Obviously
*    the segments (p1,p2) and (p2,p3) will get very close (they touch), but there would be nothing to trim between them.
*
*    If the deletion is triggered, the pruning step is complete. Since certain line segments are now gone,
*    it does not make sense to keep comparing using those (potentially deleted) line segments. The goal of this algorithm
*    is not to find the optimal simplified path, but rather to simplify it enough that it is not at risk of running out of memory.
*
*    The simplification step uses the Ramer-Douglas-Peucker algorithm. See Wikipedia for description.
*
*    Returns true if pruning occured. In this case, running cleanup() again might prune even more.
*    But if no pruning occured (returns False) then running the algorithm again will change nothing.
*/
bool Path::cleanup(){
  // pruning step
  bool pruning_occured = FALSE;
  for ( int i = 0; i <= last_index; i++ ){
    for ( int j = last_index - 1; j > i + 1; j--){
      distance_point dp = segment_segment_dist(path[i], path[i+1], path[j], path[j+1]);
      if ( dp->distance <= PRUNING_DELTA){
        // TODO prune path. Not ideal with an array. Looks like this in python: self.path = self.path[:i+1] + [dist[1]] + self.path[j+1:]
        // probably do this with memset or maybe std::move
        pruning_occured = True;
        goto simplification_step; // best way I could think of to break out of nested loops. sorry.
      }
    }
  }
  simplification_step:
  rdp(Vector3f *path, last_index, RDP_EPSILON);

  return pruning_occured;
}
