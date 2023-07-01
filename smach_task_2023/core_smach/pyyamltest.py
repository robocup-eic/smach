from geometry_msgs.msg import Pose, Point, Quaternion
import yaml

# x = yaml.load("""
# !!python/object/apply:geometry_msgs.msg.Pose
# - !!python/tuple [1.0, 2.0, 3.0]
# - !!python/tuple [0.0, 0.0, 0.0, 1.0]
# """,
# Loader=yaml.Loader)
# print(x)

# y = Pose(Point(1,2,3), Quaternion(0,0,0,1))
# print(yaml.dump(y))


# stream = open("test.yaml", "w")
# yaml.dump(z, stream)
# stream.close()


# test = yaml.load(
# '''
# !!python/object/new:geometry_msgs.msg._Point.Point
# x: 1
# y: 2
# z: 3
# ''',
# Loader=yaml.Loader)
# print(test)
# z = yaml.load("""
# !!python/object/new:geometry_msgs.msg._Pose.Pose
# state:
# - !!python/object/new:geometry_msgs.msg._Point.Point
#   state:
#   - 1
#   - 2
#   - 3
# - !!python/object/new:geometry_msgs.msg._Quaternion.Quaternion
#   state:
#   - 0
#   - 0
#   - 0
#   - 1
# """,
# Loader=yaml.Loader)
# print(z)