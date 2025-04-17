#include "Ball.h"

Ball::Ball(Transform& t, Transform& g, int i, float f, int port) : Entity(t, i, f, port), goal(g) {
}