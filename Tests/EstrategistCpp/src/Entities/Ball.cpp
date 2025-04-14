#include "Ball.h"

Ball::Ball(Transform& t, Transform& g, int i, float f) : Entity(t, i, f), goal(g) {
}