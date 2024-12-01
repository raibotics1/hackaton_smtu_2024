from MotionNode import get_dist_course
from numpy import array


assert get_dist_course(array([1, 0])) == (1.0, 0.0)
assert get_dist_course(array([0, 1])) == (1.0, 90.0)
assert get_dist_course(array([-1, 1])) == (1.4142135623730951, 135.0)
assert get_dist_course(array([0, -1])) == (1.0, 270.0)
assert get_dist_course(array([-1, -1])) == (1.4142135623730951, 225.0)
assert get_dist_course(array([1, -1])) == (1.4142135623730951, 315.0)