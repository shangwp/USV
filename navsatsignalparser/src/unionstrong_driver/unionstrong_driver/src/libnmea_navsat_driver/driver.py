import math
import rospy
import lcm
import tf
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference, Imu
from geometry_msgs.msg import TwistStamped

from libnmea_navsat_driver.checksum_utils import check_nmea_checksum
import libnmea_navsat_driver.parser
from obu_lcm import ins_info

groll=0
gpitch=0
gheading=0
gyro_x=0
gyro_y=0
gyro_z=0
gacc_x=0
gacc_y=0
gacc_z=0
ve=0
vn=0
vu=0
class RosNMEADriver(object):
    def __init__(self):
	#f = open('test.txt','a+')
        self.fix_pub = rospy.Publisher('/unionstrong/gpfpd', NavSatFix, queue_size=1)
	self.imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=1)
        self.vel_pub = rospy.Publisher('vel', TwistStamped, queue_size=1)
        self.time_ref_pub = rospy.Publisher('time_reference', TimeReference, queue_size=1)

        self.time_ref_source = rospy.get_param('~time_ref_source', None)
        self.use_RMC = rospy.get_param('~useRMC', False)


    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        #if not check_nmea_checksum(nmea_string):
        #    rospy.logwarn("Received a sentence with an invalid checksum. " +
        #                  "Sentence was: %s" % repr(nmea_string))
        #    return False
        parsed_sentence = libnmea_navsat_driver.parser.parse_nmea_sentence(nmea_string)
	#print parsed_sentence
        if not parsed_sentence:
            rospy.logdebug("Failed to parse NMEA sentence. Sentece was: %s" % nmea_string)
            return False
        if timestamp:
            current_time = timestamp
        else:
            current_time = rospy.get_rostime()
        current_fix = NavSatFix()
        current_fix.header.stamp = current_time
        current_fix.header.frame_id = frame_id
	global groll,gpitch,gheading,gyro_x,gyro_y,gyro_z,gacc_x,gacc_y,gacc_z,ve,vn,vu
        if 'FPD' in parsed_sentence:
            data = parsed_sentence['FPD']
            current_fix.latitude = data['latitude']
            current_fix.longitude = data['longitude']
            current_fix.altitude = data['altitude']
            gheading = float(data['heading'])
            groll = float(data['roll'])
            gpitch = float(data['pitch'])
            status = data['status']
            ve = float(data['ve'])
            vn = float(data['vn'])
            vu = float(data['vu'])
            print 'latitude: ', current_fix.latitude, ' longitude: ', current_fix.longitude, 'heading: ', gheading, ' status: ', status
        elif 'IMU' in parsed_sentence:
	    data1 = parsed_sentence['IMU']
            gyro_x = float(data1['gyro_x'])
            gyro_y = float(data1['gyro_y'])
            gyro_z = float(data1['gyro_z'])
            gacc_x = float(data1['acc_x'])
            gacc_y = float(data1['acc_y'])
            gacc_z = float(data1['acc_z'])

            print 'Imu', 'acc_x: ', gacc_x, ' acc_y: ', gacc_y, ' acc_z: ', gacc_z
	else:
            return False
        final_fix = NavSatFix()
        final_fix.status.status = NavSatStatus.STATUS_FIX
        final_fix.status.service = NavSatStatus.SERVICE_GPS
        final_fix.header.stamp = rospy.get_rostime()
        final_fix.header.frame_id = frame_id
        final_fix.longitude = float(current_fix.longitude)
        final_fix.latitude = float(current_fix.latitude)
        final_fix.altitude = float(current_fix.altitude)
        final_fix.position_covariance[1] = float(gheading)
        final_fix.position_covariance[2] = float(groll)
        final_fix.position_covariance[3] = float(gpitch)
        final_fix.position_covariance[0] = float(current_fix.position_covariance[0])
        final_fix.position_covariance[4] = float(current_fix.position_covariance[4])
        final_fix.position_covariance[8] = float(current_fix.position_covariance[8])
	final_fix.position_covariance[5] = float(ve)
	final_fix.position_covariance[6] = float(vn)
	final_fix.position_covariance[7] = float(vu)
	#context = str(current_fix.longitude)+','+str(current_fix.latitude)+','+ str(current_fix.altitude)+','
	#f.write(context)
	#contest = str(final_fix.position_covariance[1])+','+str(final_fix.position_covariance[5])+','+str(final_fix.position_covariance[6])+'\n'
        self.fix_pub.publish(final_fix)

	imudata = Imu()
        imudata.header.stamp = rospy.get_rostime()
        imudata.header.frame_id = frame_id
        quaternion = tf.transformations.quaternion_from_euler(groll,gheading,gpitch)
        imudata.orientation.x = quaternion[0]
        imudata.orientation.y = quaternion[1]
        imudata.orientation.z = quaternion[2]
        imudata.orientation.w = quaternion[3]
        imudata.angular_velocity.x = gyro_x
        imudata.angular_velocity.y = gyro_y
        imudata.angular_velocity.z = gyro_z
        imudata.linear_acceleration.x = gacc_x
        imudata.linear_acceleration.y = gacc_y
        imudata.linear_acceleration.z = gacc_z

        self.imu_pub.publish(imudata)
	
    
    """Helper method for getting the frame_id with the correct TF prefix"""

    @staticmethod
    def get_frame_id():
        frame_id = rospy.get_param('~frame_id', 'gps')
        if frame_id[0] != "/":
            """Add the TF prefix"""
            prefix = ""
            prefix_param = rospy.search_param('tf_prefix')
            if prefix_param:
                prefix = rospy.get_param(prefix_param)
                if prefix[0] != "/":
                    prefix = "/%s" % prefix
            return "%s/%s" % (prefix, frame_id)
        else:
            return frame_id
