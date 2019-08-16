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


class FLO_DB(object):
    def __init__(self):
        rospy.init_node('db_manager')

        self.db_path = rospy.get_param("database_location")

        serv_get_pose_id = rospy.Service(
            'get_pose_id', GetPoseID, self.get_pose_id)
        serv_set_pose = rospy.Service('set_pose', SetPose, self.set_pose)

        rospy.loginfo('Node up, services ready')

        rospy.spin()

    def get_pose_id(self, request):
        db = DB(self.db_path)
        curs = db.ex('select * from poses where id = ?', request.id)
        data = curs.fetchone()
        resp = GetPoseIDResponse()
        resp.pose.description = data['description']
        resp.pose.joint_names = json.loads(data['joint_names'])
        resp.pose.joint_positions = json.loads(data['joint_positions'])
        return resp

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
        """Set the passed in pose in 

        :param request:
        """
        db = DB(self.db_path)
        if request.id:
            db_return = db.ex('replace into poses(id, description, joint_positions, joint_names) values (?,?,?,?)',
                              request.id,
                              request.pose.description,
                              json.dumps(request.pose.joint_positions),
                              json.dumps(request.pose.joint_names))
            updated_row = request.id
            rospy.loginfo('updated pose at id: %i', updated_row)
        else:
            db_return = db.ex('insert into poses(description, joint_positions, joint_names) values (?,?,?)',
                              request.pose.description,
                              json.dumps(request.pose.joint_positions),
                              json.dumps(request.pose.joint_names))
            updated_row = db_return.lastrowid
            rospy.loginfo('stored new pose at id: %i', udpated_row)

        return updated_row


if __name__ == "__main__":
    FLO_DB()
