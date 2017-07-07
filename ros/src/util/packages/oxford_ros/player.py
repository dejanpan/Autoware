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
from sdk.TimerProcess import TimerProcess

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
        self.firstValidId = -1
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
        
    def initializeRun (self):
#        File Collector
        fileList = [
            self.imageList[pr]['center'] 
            for pr in range(len(self.imageList)) 
                if pr>=self.firstValidId
        ]
        self.collector = FileCollector(fileList, self.readFileFunc)
        
    def close (self):
        print ("Closing images")
        self.collector.close()

    def _getEvents (self):
        eventList = [ 
            {'timestamp':self.imageList[i]['timestamp'], 'id':i} 
                for i in range(len(self.imageList)) 
        ]
        return eventList
    
    def _passEvent (self, timestamp, eventId, publish=True):
        image_ctr = self.collector.pick()
#         image_ctr = cv2.imread(imageTarget['center'], cv2.IMREAD_ANYCOLOR)
#         image_ctr = self.imagePostProcessing(image_ctr)
        if (image_ctr is None):
            return
        
        msg = self.cvbridge.cv2_to_imgmsg(image_ctr, 'bgr8')
        msg.header.stamp = rospy.Time.from_sec (timestamp)
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
        if (image is None):
            print ("Image is empty: {}".format(path))
            return None
        return self.imagePostProcessing(image)


class Lidar3Player:
    _lidarName = 'ldmrs'
    
    def __init__ (self, dataset):
        self.firstValidId = -1
        self.lidarFileSet = dataset.getMainLidar()
        self.publisher = rospy.Publisher ('/oxford/'+Lidar3Player._lidarName, PointCloud2, queue_size=10)
    
    def close (self):
        print ("Closing {} set".format(self._lidarName))
        self.collector.close()
    
    def initializeRun (self):
        lidarFileList = [
            self.lidarFileSet[p]['path']
            for p in range(len(self.lidarFileSet))
                if p >= self.firstValidId
        ]
        self.collector = FileCollector(lidarFileList, self.readFileFunc)
    
    def _getEvents (self):
        eventList = [ {
            'timestamp': self.lidarFileSet[i]['timestamp'],
            'id': i
        } for i in range(len(self.lidarFileSet)) ]
        return eventList
    
    def _passEvent (self, timestamp, eventId, publish=True):
        scan = self.collector.pick()
        header = std_msgs.msg.Header(
            stamp=rospy.Time.from_sec(timestamp), 
            frame_id=Lidar3Player._lidarName)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=16, datatype=PointField.FLOAT32, count=1)
        ]
        msg = pcl2.create_cloud(header, fields, scan)
        if (publish):
            self.publisher.publish(msg)
        else:
            return msg
        
    def readFileFunc (self, path):
        scan = np.fromfile(path, np.double)
        return scan.reshape ((len(scan) // 3,3)).astype(np.float32)
        

class Lidar2Player (Lidar3Player):
    _lidarName = 'lms_front'
    
    def __init__ (self, dataset):
        self.firstValidId = -1
        self.lidarFileSet = dataset.getLidar2D('front')
        self.publisher = rospy.Publisher ('/oxford/'+self._lidarName, PointCloud2, queue_size=10)
    
    def _passEvent (self, timestamp, eventId, publish=True):
        scan = self.collector.pick()
#         scan = scan[:,0:2]
        header = std_msgs.msg.Header(
            stamp=rospy.Time.from_sec(timestamp), 
            frame_id=self._lidarName)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='i', offset=16, datatype=PointField.FLOAT32, count=1)
        ]
        msg = pcl2.create_cloud(header, fields, scan)
        if (publish):
            self.publisher.publish(msg)
        else:
            return msg
        
        




class PosePlayer:
    def __init__ (self, dataset):
        self.firstValidId = -1
        self.poses = dataset.getIns()
        self.publisher = rospy.Publisher ('/oxford/pose', PoseMsg, queue_size=1)
        self.tfb = TransformBroadcaster()
        
    def close(self):
        pass
    
    def initializeRun (self):
        pass
    
    def _getEvents (self):
        eventList = [{'timestamp':self.poses[p,0], 'id':p} for p in range(len(self.poses))]
        return eventList
    
    def _passEvent (self, timestamp, eventId, publish=True):
        poseRow = self.poses[eventId]
        curPose = PosePlayer.createPoseFromRPY(
            poseRow[1], poseRow[2], poseRow[3], poseRow[4], -poseRow[5], -poseRow[6])
        curPose.header.stamp = rospy.Time.from_sec(timestamp)
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
                rospy.Time.from_sec(timestamp),
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


class PlayerControl:
    def __init__ (self, datadir, rate=1.0, start=0.0):
        self.rate = float(rate)
        self.eventList = []
        self.dataset = sdk.Dataset(datadir)
        self.players = []
        self.startTime = start
        
    def add_data_player (self, _dataPlayer):
        if (_dataPlayer is None):
            return
        self.players.append(_dataPlayer)
    
    def run (self):
        self.initRun()
        isPause = NonblockingKeybInput()
        
        while (True):
            # Wait for a timer event
            self.timer.eventNotification.wait()
            self.timer.eventNotification.clear()
            curEvent = self.eventList[self.timer.currentEventTimerId]
#             ct = curEvent['timestamp']
            ct = self.timer.currentTimestamp
            curEvent['object']._passEvent (ct, curEvent['id'])
            
            if (rospy.is_shutdown()):
                break
            if isPause.spacePressed():
                self.timer.pause()
                isPause.readUntilSpace()
                self.timer.resume()
        
        isPause.setBlock()
        for p in self.players:
            p.close()
        self.timer.close()
        
    def initRun (self):
        
        # Build list of events
        for player in self.players:
            eventsInThis = player._getEvents ()
            if len(eventsInThis)==0:
                continue
            for evt in eventsInThis:
                e = {'timestamp': evt['timestamp'], 'id':evt['id'], 'object':player}
                self.eventList.append(e)
        self.eventList.sort(key=lambda e: e['timestamp'])
        if self.startTime == 0.0:
            for player in self.players:
                player.firstValidId = 0

        else:    
            validEvents = []
            start=self.eventList[0]['timestamp']
            for evt in self.eventList:
                if evt['timestamp'] < start + self.startTime:
                    continue
                else:
                    if evt['object'].firstValidId == -1:
                        evt['object'].firstValidId = evt['id']
                    validEvents.append(evt)
            
            self.eventList = validEvents

        # Tell data players to initialize
        for player in self.players:
            player.initializeRun()
        
        # Tell timer to initialize. Put a delay 1.s 
        self.timer = TimerProcess (
            [self.eventList[i]['timestamp'] for i in range(len(self.eventList))], 
            self.eventList[0]['timestamp']-1.0, 
            self.rate)
        
        
if __name__ == '__main__' :
    import sys
    import argparse
    
    argsp = argparse.ArgumentParser('Oxford ROS Player')
    argsp.add_argument('--dir', type=str, default=None, help='Directory of Oxford dataset')
    argsp.add_argument('--rate', type=float, default=1.0, help='Speed up/Slow down by rate factor')
    argsp.add_argument('--start', type=float, default=0.0, help='Start SEC seconds into dataset')
    args, unknown_args = argsp.parse_known_args()
    
    rospy.init_node('oxford_player', anonymous=True)
    player = PlayerControl (args.dir, rate=args.rate, start=args.start)
    poses = PosePlayer (player.dataset)
    images = ImagePlayer(player.dataset)
    lidar3d = Lidar3Player (player.dataset)
    lidarfront = Lidar2Player (player.dataset)
    player.add_data_player(poses)
    player.add_data_player(images)
    player.add_data_player(lidar3d)
    player.add_data_player(lidarfront)
    
    print ("[SPACE] to pause, [Ctrl+C] to break")
    player.run()
    print ("Done")

