#!/usr/bin/env python
import copy
import rospy
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import tf

# All coordinates are in relation to this frame, most likely this should be the robot base
REFERENCE_FRAME = "/segment_0"


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('test_environment_publisher')

    # Create the publisher for the collisionObject
    pub_col = rospy.Publisher("/collision_object", CollisionObject)

def publish_hovering_cross():
    '''
    Publishes a hovering cross in front of the robot
    '''

    static_scene = _init_collision_object("cross")
    _add_part((0.4, 0, 0.3), (0.4, 0.1, 0.1), static_scene)
    _add_part((0.4, 0, 0.3), (0.1, 0.4, 0.1), static_scene)

    pub_col.publish(static_scene)
    rospy.loginfo("Published static scene")
    rospy.sleep(0.2)

def publish_moving_obstacle():
    '''
    publishes a slowly moving obstacle, which should limit the ellbow options for the hovering cross
    This method is synchronous and only exits when the movement is cancelled via ctrl + c, after which the command line
    has to be restarted
    '''
    moving_obstacle = _init_collision_object("moving_obstacle")
    _add_part((0.2, 0, 0.7), (0.1, 0.1, 0.3), moving_obstacle)

    move_object(moving_obstacle, (0, 0.4, 0))

def publish_static_scene_flat_front():
    '''
    Publishes static obstacles in front of the robot
    '''

    
    static_scene = _init_collision_object("static_scene")
    _add_part((0.35, 0, 0.1), (0.5, 0.1, 0.2), static_scene)
    _add_part((0.35, 0.3, 0.1), (0.5, 0.1, 0.2), static_scene)
    _add_part((0.35, -0.3, 0.1), (0.5, 0.1, 0.2), static_scene)

    pub_col.publish(static_scene)
    rospy.loginfo("Published static scene")

    # Sometimes this sleep is necessary for the planning-scene to catch up
    rospy.sleep(0.2)


def publish_static_scene_upright_front():
    '''
    Publishes static obstacles upright in front of the robot
    '''

    static_scene = _init_collision_object("static_scene_2")
    _add_part((0.35, 0, 0.1), (0.1, 0.1, 0.5), static_scene)
    _add_part((0.35, 0.3, 0.1), (0.1, 0.1, 0.5), static_scene)
    _add_part((0.35, -0.3, 0.1), (0.1, 0.1, 0.5), static_scene)

    pub_col.publish(static_scene)
    rospy.loginfo("Published static scene")

    # Sometimes this sleep is necessary for the planning-scene to catch up
    rospy.sleep(0.2)


def remove_object(id):
    '''
    Removes a collisionObject from the planning_scene
    '''

    colObj = _init_collision_object(id)
    colObj.operation = colObj.REMOVE

    pub_col.publish(colObj)


def move_object(colObj, (x_range, y_range, z_range)):
    '''
    Moves a CollisionObject in the workspace
    The movement starts in -range/2 and ends at range/2
    It is then repeated from -range/2 until Ctrl + C cancels the movement
    '''

    rospy.logwarn("Moving an Object. Ctrl + C cancels the movement, but you need to restart the python command line to issue further commands after cancelling the movement")
    published_object = copy.deepcopy(colObj)
    i = 0.0
    j = 0.0
    k = 0.0

    while not rospy.is_shutdown():
        i += 0.005
        j += 0.005
        k += 0.005

        if i > x_range * 0.5:
            i = -(x_range * 0.5)
        if j > y_range * 0.5:
            j = -(y_range * 0.5)
        if k > z_range * 0.5:
            k = -(z_range * 0.5)

        for p in published_object.primitive_poses:
            index = published_object.primitive_poses.index(p)
            p.position.x = colObj.primitive_poses[index].position.x + i
            p.position.y = colObj.primitive_poses[index].position.y + j
            p.position.z = colObj.primitive_poses[index].position.z + k

        pub_col.publish(published_object)
        rospy.sleep(0.7)
    

def _init_collision_object(name):
    '''
    Sets up the collision object with the given name as ID
    '''
    
    colObj = CollisionObject()
    colObj.id = name # The name by which the object is identified, necessary to (re)move it
    colObj.header.frame_id = REFERENCE_FRAME # In relation to which frame the object is published
    colObj.operation = colObj.ADD # Defines what operation the planning_scene should do with the object. ADDing moves the object, if it's already part of the scene

    return colObj


def _add_part((x_pos, y_pos, z_pos), (x_size, y_size, z_size), collisionObject):
    '''
    Creates a box of the given size at the given position and adds it to the collisionObject
    '''
    # Create the seperate parts of the box
    partPrimitive = SolidPrimitive()
    partPrimitive.type = partPrimitive.BOX
    # Append dimensions of the box in the xyz order
    partPrimitive.dimensions.append(x_size)
    partPrimitive.dimensions.append(y_size)
    partPrimitive.dimensions.append(z_size)

    partPose = Pose()
    partPose.position.x = x_pos
    partPose.position.y = y_pos
    partPose.position.z = z_pos

    # We don't change the pose of the boxes
    partPose.orientation.x = 0
    partPose.orientation.y = 0
    partPose.orientation.z = 0
    partPose.orientation.w = 0
    
    collisionObject.primitives.append(partPrimitive)
    collisionObject.primitive_poses.append(partPose)


