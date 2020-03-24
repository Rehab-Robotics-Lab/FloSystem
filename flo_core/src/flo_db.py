#! /usr/bin/env python

from __future__ import print_function

import json
from os import path
import rospy


from flo_core_defs.srv import GetPoseID, GetPoseIDResponse
from flo_core_defs.srv import SetPose
from flo_core_defs.srv import SearchPose, SearchPoseResponse
from flo_core_defs.msg import Pose
from flo_core_defs.srv import SetPoseSeq
from flo_core_defs.srv import GetPoseSeqID, GetPoseSeqIDResponse
from flo_core_defs.srv import SearchPoseSeq, SearchPoseSeqResponse
from flo_core_defs.srv import SetUtterance, SetUtteranceResponse
from flo_core_defs.srv import SearchUtterance, SearchUtteranceResponse
from flo_core_defs.msg import PoseSeq
from flo_core_defs.msg import DBUpdate

import mutagen

from db import DB

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

        self.db_updates = rospy.Publisher('flo_db_updates', DBUpdate)

        self.db_path = rospy.get_param("database_location",
                                       path.expanduser('~/db/flo.db'))

        rospy.Service('get_pose_id', GetPoseID, self.__get_pose_id)
        rospy.Service('set_pose', SetPose, self.__set_pose)
        rospy.Service('search_pose', SearchPose, self.__search_pose)
        rospy.Service('set_pose_seq', SetPoseSeq, self.__set_pose_seq)
        rospy.Service('get_pose_seq_id', GetPoseSeqID, self.__get_pose_seq_id)
        rospy.Service('search_pose_seq', SearchPoseSeq, self.__search_pose_seq)
        rospy.Service('search_utterance', SearchUtterance,
                      self.__search_utterance)
        rospy.Service('set_utterance', SetUtterance, self.__set_utterance)

        rospy.loginfo('Node up, services ready')

        rospy.spin()

    def __get_pose_id(self, request):
        """Return a pose and its decription using its ID

        :param request: The GetPoseID service request
        """
        db = DB(self.db_path)  # pylint:disable=invalid-name
        curs = db.ex('select * from poses where id = ?', request.id)
        data = curs.fetchone()
        if data:
            resp = GetPoseIDResponse()
            resp.pose.description = data['description']
            resp.pose.joint_names = json.loads(data['joint_names'])
            resp.pose.joint_positions = json.loads(data['joint_positions'])
            return resp
        raise rospy.ServiceException('That ID does not exist')

    def __search_pose(self, request):
        """search for a pose in the database using descriptions

        Args:
            request: the service request with the string to look with

        Returns:
            the service response
        """
        db = DB(self.db_path)  # pylint: disable=invalid-name
        resp = SearchPoseResponse()
        for row in db.ex('select * from poses where description like ?',
                         '%'+request.search+'%'):
            new_pose = Pose()
            new_pose.description = row['description']
            new_pose.joint_names = json.loads(row['joint_names'])
            new_pose.joint_positions = json.loads(row['joint_positions'])

            resp.poses.append(new_pose)
            resp.ids.append(row['id'])
        return resp

    @staticmethod
    def __clean_pose_names(name_list, side):
        """Clean the pose names in a list by removing the leading side word.
        Ex: will take [left_shoulder_abduction, left_elbow_flexion] and return
            [shoulder_abduction, elbow_flexion]

        :param name_list: The list of joint names to clean
        :param side: The side which is present in the joint names
        """
        return [itm[len(side)+1:] for itm in name_list]

    def __set_pose(self, request):
        """Set the passed in pose in the database. If the default value for the
        id of 0 is used, a new entry will be created. If the value of an existing
        row's id is passed, that row will be replaced with the new data. If
        a non-zero id is passed which does not exist in the database, an error
        will be raised.

        :param request: The SetPose service message request
        """
        db = DB(self.db_path)  # pylint: disable=invalid-name
        update = DBUpdate()
        if not len(request.pose.joint_positions) == len(request.pose.joint_names):
            raise rospy.ServiceException(
                'the length of the pose values and names are not consistent')

        if request.id:
            curs = db.ex('select id from poses where id = ?', request.id)
            data = curs.fetchone()
            if data:
                db_return = db.ex(
                    'replace into poses(id, description, joint_positions, joint_names)' +
                    ' values (?,?,?,?)',
                    request.id,
                    request.pose.description,
                    json.dumps(request.pose.joint_positions),
                    json.dumps(request.pose.joint_names))
                updated_row = request.id
                rospy.loginfo('updated pose at id: %i', updated_row)
                update.command = DBUpdate.UPDATE
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
            update.command = DBUpdate.CREATE

        update.table = DBUpdate.POSES
        update.value = json.dumps(
            {'description': request.pose.description,
             'joint_positions': request.pose.joint_positions,
             'joint_names': request.pose.joint_names})
        self.db_updates.publish(update)
        update.id = updated_row
        return updated_row

    def __set_pose_seq(self, request):
        """set_pose_seq

        Args:
            request: The service request

        Returns:
            The response.
        """
        db = DB(self.db_path)  # pylint: disable=invalid-name
        seq = request.sequence
        update = DBUpdate()

        if not len(seq.pose_ids) == len(seq.times) == len(seq.arms):
            raise rospy.ServiceException(
                'the length of the pose ids, times, and arms are not consistent')

        for pose_id in seq.pose_ids:
            curs = db.ex('select id from poses where id = ?', pose_id)
            data = curs.fetchone()
            if not data:
                raise rospy.ServiceException(
                    'The pose id: {} does not exist'.format(pose_id))

        if request.id:
            curs = db.ex(
                'select id from pose_sequences where id = ?', request.id)
            data = curs.fetchone()
            if data:
                db_return = db.ex(
                    'replace into pose_sequences(id, times, pose_ids, total_time,' +
                    ' arms, description) values (?,?,?,?,?,?)',
                    request.id,
                    json.dumps(seq.times),
                    json.dumps(seq.pose_ids),
                    seq.total_time,
                    json.dumps(seq.arms),
                    seq.description
                )
                updated_row = request.id
                update.command = DBUpdate.UPDATE
            else:
                raise rospy.ServiceException(
                    'The selected row does not exist, you cannot update it')
        else:
            db_return = db.ex(
                'insert into pose_sequences(times, pose_ids, total_time, arms, description)' +
                ' values (?,?,?,?,?)',
                json.dumps(seq.times),
                json.dumps(seq.pose_ids),
                seq.total_time,
                json.dumps(seq.arms),
                seq.description
            )
            updated_row = db_return.lastrowid
            update.command = DBUpdate.CREATE

        update.id = updated_row
        update.table = DBUpdate.POSE_SEQUENCES
        update.value = json.dumps({
            'times': seq.times,
            'pose_ids': seq.pose_ids,
            'total_time': seq.total_time,
            'arms': seq.arms,
            'description': seq.description
        })
        self.db_updates.publish(update)

        return updated_row

    def __get_pose_seq_id(self, request):
        """Get a pose sequence given its id

        Args:
            request: the service request

        Returns:
            The service response
        """
        db = DB(self.db_path)  # pylint: disable=invalid-name
        curs = db.ex('select * from pose_sequences where id = ?', request.id)
        data = curs.fetchone()
        if data:
            resp = GetPoseSeqIDResponse()
            resp.sequence.description = data['description']
            resp.sequence.pose_ids = json.loads(data['pose_ids'])
            resp.sequence.times = json.loads(data['times'])
            resp.sequence.arms = json.loads(data['arms'])
            resp.sequence.total_time = data['total_time']
            return resp
        raise rospy.ServiceException('That ID does not exist')

    def __search_pose_seq(self, request):
        """Search for a pose sequence in the database using a string of the description

        Args:
            request: the service request

        Returns:
            The service response
        """
        db = DB(self.db_path)  # pylint:disable=invalid-name
        resp = SearchPoseSeqResponse()
        for row in db.ex('select * from pose_sequences where description like ?',
                         '%'+request.search+'%'):
            new_pose_seq = PoseSeq()
            new_pose_seq.description = row['description']
            new_pose_seq.pose_ids = json.loads(row['pose_ids'])
            new_pose_seq.times = json.loads(row['times'])
            new_pose_seq.arms = json.loads(row['arms'])
            new_pose_seq.total_time = row['total_time']

            resp.sequences.append(new_pose_seq)
            resp.ids.append(row['id'])
        return resp

    def __search_utterance(self, request):
        """Search for a requested utterance in the DB

        Args:
            request: The SearchUtteranceRequest from the service

        Returns: the SearchUtteranceResponse that the service will send out
        """
        db = DB(self.db_path)  # pylint: disable=invalid-name
        resp = SearchUtteranceResponse()
        for row in db.ex('select * from utterances where text like ?',
                         '%'+request.search+'%'):
            resp.ids.append(row['id'])
            resp.texts.append(row['text'])
            resp.metadatas.append(row['metadata'])
            resp.length.append(row['length'])
        return resp

    def __set_utterance(self, request):
        """Set an utterance in the Database

        Updates (if given an ID) or creates a new utterance (if not given and ID)
        based on the request passed from the action server

        Args:
            request: The action server request

        Returns: The action server response
        """
        db = DB(self.db_path)  # pylint: disable=invalid-name
        update = DBUpdate()
        # Find the length of the utterance:
        mut_d = mutagen.File(request.filename)
        time_length = mut_d.info.length
        if request.id:
            curs = db.ex('select id from utterances where id = ?', request.id)
            data = curs.fetchone()
            if data:
                db_return = db.ex(
                    'replace into utterances(id, text, length, metadata) values (?,?,?,?)',
                    request.id,
                    request.text,
                    time_length,
                    request.metadata
                )
                updated_row = request.id
                rospy.loginfo('updated utterance at id: %i', updated_row)
                update.command = DBUpdate.UPDATE
            else:
                rospy.logerr('Attempt to change a non-existant row')
                raise rospy.ServiceException(
                    'The selected row does not exist, you cannot update it')
        else:
            db_return = db.ex(
                'insert into utterances(text, length, metadata) values (?,?,?)',
                request.text,
                time_length,
                request.metadata
            )
            updated_row = db_return.lastrowid
            rospy.loginfo('stored new utterance at id: %i', updated_row)
            update.command = DBUpdate.CREATE

        update.id = updated_row
        update.table = DBUpdate.UTTERANCES
        update.command = json.dumps({
            'text': request.text,
            'length': time_length,
            'metadata': request.metadata
        })
        resp = SetUtteranceResponse(id=updated_row, time=time_length)
        return resp


if __name__ == "__main__":
    FloDb()
