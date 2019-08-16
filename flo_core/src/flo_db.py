#! /usr/bin/env python

from __future__ import print_function

import sqlite3
import pdb
import json
import rospy
import json

from db import DB

from flo_core.srv import GetPoseID, GetPoseIDResponse
from flo_core.srv import SetPose, SetPoseResponse

# Needs to be able to:
# - search for pose by id
# - search for pose by string
# - gep pose seq by id
# - search for pose seq by string
# - add pose <- if ID specified, replace
# - add pose seq <- if ID specified, replace


class FloDb(object):
    """A class to manage all of the database interactions for flo"""

    def __init__(self):
        """Setup the node and the services, load the database location.
        Note, we do not actually connect to the database because the threads
        created by rospy spin will break the connection
        """

        rospy.init_node('db_manager')

        self.db_path = rospy.get_param("database_location")

        rospy.Service('get_pose_id', GetPoseID, self.get_pose_id)
        rospy.Service('set_pose', SetPose, self.set_pose)

        rospy.loginfo('Node up, services ready')

        rospy.spin()

    def get_pose_id(self, request):
        """Return a pose and its decription using its ID

        :param request: The GetPoseID service request
        """
        db = DB(self.db_path)
        curs = db.ex('select * from poses where id = ?', request.id)
        data = curs.fetchone()
        if data:
            resp = GetPoseIDResponse()
            resp.pose.description = data['description']
            resp.pose.joint_names = json.loads(data['joint_names'])
            resp.pose.joint_positions = json.loads(data['joint_positions'])
            return resp
        else:
            raise rospy.ServiceException('That ID does not exist')

    @staticmethod
    def clean_pose_names(name_list, side):
        """Clean the pose names in a list by removing the leading side word.
        Ex: will take [left_shoulder_abduction, left_elbow_flexion] and return
            [shoulder_abduction, elbow_flexion]

        :param name_list: The list of joint names to clean
        :param side: The side which is present in the joint names
        """
        return [itm[len(side)+1:] for itm in name_list]

    def set_pose(self, request):
        """Set the passed in pose in the database. If the default value for the
        id of 0 is used, a new entry will be created. If the value of an existing
        row's id is passed, that row will be replaced with the new data. If
        a non-zero id is passed which does not exist in the database, an error
        will be raised.

        :param request: The SetPose service message request
        """
        db = DB(self.db_path)
        if not (len(request.pose.joint_positions) == len(request.pose.joint_names)):
            raise rospy.ServiceException(
                'the length of the pose values and names are not consistent')

        if request.id:
            curs = db.ex('select id from poses where id = ?', request.id)
            data = curs.fetchone()
            if data:
                db_return = db.ex(
                    'replace into poses(id, description, joint_positions, joint_names) values (?,?,?,?)',
                    request.id,
                    request.pose.description,
                    json.dumps(request.pose.joint_positions),
                    json.dumps(request.pose.joint_names))
                updated_row = request.id
                rospy.loginfo('updated pose at id: %i', updated_row)
            else:
                rospy.logerr('Attempt to change a non-existant row')
                raise rospy.ServiceException(
                    'The selected row does not exist, you cannot update it')
        else:
            db_return = db.ex(
                'insert into poses(description, joint_positions, joint_names) values (?,?,?)',
                request.pose.description,
                json.dumps(request.pose.joint_positions),
                json.dumps(request.pose.joint_names))
            updated_row = db_return.lastrowid
            rospy.loginfo('stored new pose at id: %i', updated_row)

        return updated_row


if __name__ == "__main__":
    FloDb()
