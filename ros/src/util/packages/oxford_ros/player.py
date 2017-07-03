#!/usr/bin/python

import sdk
import time
import rospy
import yaml
import cv2
import cv_bridge
import rospkg
from tf import transformations
from tf import TransformBroadcaster
import numpy as np
from sdk.nonblocking import NonblockingKeybInput
from sdk.FileCollector import FileCollector

from geometry_msgs.msg import PoseStamped as PoseMsg
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from rosgraph_msgs.msg import Clock
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
import std_msgs



class ImagePlayer:
    def __init__ (self, dataset):
        self.publisher = rospy.Publisher ('/oxford/image', ImageMsg, queue_size=1)
        self.imageList = dataset.getStereo()
        pkgpack = rospkg.RosPack()
        path = pkgpack.get_path('oxford_ros')
#         self.cameraModel = sdk.CameraModel (path+'/models', sdk.CameraModel.cam_stereo_center)
        
        calib_file = file(path+'/calibration_files/bb_xb3_center.yaml')
        conf = yaml.load(calib_file)
        self.camera_matrix = np.reshape(conf['camera_matrix']['data'], (3,3))
        self.projection_matrix = np.reshape(conf['projection_matrix']['data'], (3,4))
        self.distortion_coefs = np.array(conf['distortion_coefficients']['data'])
#         self.calibrator = cv2.cv.Load(path+'/calibration_files/bb_xb3_center.yaml')
        self.cvbridge = cv_bridge.CvBridge()
        
#        File Collector
#         fileList = [pr['center'] for pr in self.imageList]
#         self.collector = FileCollector(fileList, self.readFileFunc)

    def _getEvents (self):
        eventList = [ {'timestamp':self.imageList[i]['timestamp'], 'id':i} for i in range(len(self.imageList)) ]
        return eventList
    
    def _passEvent (self, timestamp, eventId, publish=True):
        imageTarget = self.imageList[eventId]
        
#         image_ctr = self.collector.pick()
        image_ctr = cv2.imread(imageTarget['center'], cv2.IMREAD_ANYCOLOR)
        image_ctr = self.imagePostProcessing(image_ctr)
        
        msg = self.cvbridge.cv2_to_imgmsg(image_ctr, 'bgr8')
        msg.header.stamp = rospy.Time.from_sec (imageTarget['timestamp'])
        if (publish):
            self.publisher.publish(msg)
        else:
            return msg
        
    def imagePostProcessing (self, imageMat):
        imageMat = cv2.cvtColor(imageMat, cv2.COLOR_BAYER_GR2BGR)
        # Using camera matrix
        imageMat = cv2.undistort(imageMat, self.camera_matrix, self.distortion_coefs)
        # Using LUT
#         image_ctr = self.cameraModel.undistort (image_ctr)
        return imageMat
    
    def readFileFunc (self, path):
        image = cv2.imread(path, cv2.IMREAD_ANYCOLOR)
        return self.imagePostProcessing(image)


class LidarPlayer:
    def __init__ (self, dataset):
        self.lidarFileSet = dataset.getMainLidar()
        self.publisher = rospy.Publisher ('/oxford/ldmrs', PointCloud2, queue_size=10)
        pass
    
    def _getEvents (self):
        eventList = [ {
            'timestamp': self.lidarFileSet[i]['timestamp'],
            'id': i
        } for i in range(len(self.lidarFileSet)) ]
        return eventList
    
    def _passEvent (self, timestamp, eventId, publish=True):
        lidarFile = self.lidarFileSet[eventId]
        scan = np.fromfile(lidarFile['path'], np.double)
        scan = scan.reshape((len(scan) // 3, 3))        
        header = std_msgs.msg.Header(
            stamp=rospy.Time.from_sec(timestamp), 
            frame_id='ldmrs')
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT64, count=1),
            PointField(name='y', offset=8, datatype=PointField.FLOAT64, count=1),
            PointField(name='z', offset=16, datatype=PointField.FLOAT64, count=1)
        ]
        msg = pcl2.create_cloud(header, fields, scan)
        if (publish):
            self.publisher.publish(msg)
        else:
            return msg
        
    def readFileFunc (self, path):
        scan = np.fromfile(path, np.double)
        return scan.reshape ((len(scan) // 3,3))
        



class PosePlayer:
    def __init__ (self, dataset):
        self.poses = dataset.getIns()
        self.publisher = rospy.Publisher ('/oxford/pose', PoseMsg, queue_size=1)
        self.tfb = TransformBroadcaster()
    
    def _getEvents (self):
        eventList = [{'timestamp':self.poses[p,0], 'id':p} for p in range(len(self.poses))]
        return eventList
    
    def _passEvent (self, timestamp, eventId, publish=True):
        poseRow = self.poses[eventId]
        curPose = PosePlayer.createPoseFromRPY(
            poseRow[1], poseRow[2], poseRow[3], poseRow[4], poseRow[5], poseRow[6])
        curPose.header.stamp = rospy.Time.from_sec(poseRow[0])
        curPose.header.frame_id = 'world'
        if (publish):
            self.publisher.publish(curPose)
            self.tfb.sendTransform(
                (curPose.pose.position.x,
                 curPose.pose.position.y,
                 curPose.pose.position.z),
                (curPose.pose.orientation.x,
                 curPose.pose.orientation.y,
                 curPose.pose.orientation.z,
                 curPose.pose.orientation.w),
                curPose.header.stamp,
                'base_link',
                'world'
            )
        else:
            return curPose
        
    @staticmethod
    def createPoseFromRPY (x, y, z, roll, pitch, yaw):
        p = PoseMsg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        qt = transformations.quaternion_from_euler(roll, pitch, yaw)
        p.pose.orientation.x = qt[0]
        p.pose.orientation.y = qt[1]
        p.pose.orientation.z = qt[2]
        p.pose.orientation.w = qt[3]
        return p


class Player:
    def __init__ (self, datadir, rate=1.0, start=0.0):
        self.rate = float(rate)
        self.eventList = []
        self.dataset = sdk.Dataset(datadir)
        self.players = []
        self.clockPub = rospy.Publisher ('/clock', Clock, queue_size=1)
        self.startTime = start
        
    def add_data_player (self, _dataPlayer):
        if (_dataPlayer is None):
            return
        self.players.append(_dataPlayer)
    
    def run (self):
        self.initRun()
        isPause = NonblockingKeybInput()
        try:
            for i in range(len(self.eventList)):
                curEvent = self.eventList[i]
                t1 = curEvent['timestamp']
                curEvent['object']._passEvent (curEvent['timestamp'], curEvent['id'])
                if i<len(self.eventList)-1 :
                    t2 = self.eventList[i+1]['timestamp']
                    delay = (t2 - t1) / self.rate
                    self.publishClock(curEvent['timestamp'])
                    time.sleep(delay)
                if (rospy.is_shutdown()):
                    break
                if isPause.spacePressed():
                    isPause.readUntilSpace()
        except KeyboardInterrupt:
            print ("Interrupted")
        isPause.setBlock()
        
    def publishClock (self, t):
        ct = Clock()
        ct.clock = rospy.Time.from_sec(t)
        self.clockPub.publish(ct)
    
    def initRun (self):
        for player in self.players:
            eventsInThis = player._getEvents ()
            if len(eventsInThis)==0:
                continue
            for evt in eventsInThis:
                e = {'timestamp': evt['timestamp'], 'id':evt['id'], 'object':player}
                self.eventList.append(e)
        self.eventList.sort(key=lambda e: e['timestamp'])
        if self.startTime == 0.0:
            return
        validEvents = []
        start=self.eventList[0]['timestamp']
        for evt in self.eventList:
            if evt['timestamp'] < start + self.startTime:
                continue
            else:
                validEvents.append(evt)
        self.eventList = validEvents
        
        
        
if __name__ == '__main__' :
    import sys
    import argparse
    
    argsp = argparse.ArgumentParser('Oxford ROS Player')
    argsp.add_argument('--dir', type=str, default=None, help='Directory of Oxford dataset')
    argsp.add_argument('--rate', type=float, default=1.0, help='Speed up/Slow down by rate factor')
    argsp.add_argument('--start', type=float, default=0.0, help='Start SEC seconds into dataset')
    args = argsp.parse_args()
    
    rospy.init_node('oxford_player', anonymous=True)
    player = Player (args.dir, rate=args.rate, start=args.start)
    poses = PosePlayer (player.dataset)
    images = ImagePlayer(player.dataset)
#     lidar3d = LidarPlayer (player.dataset)
    player.add_data_player(poses)
    player.add_data_player(images)
#     player.add_data_player(lidar3d)
    player.run()

