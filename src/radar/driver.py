import sys
from time import sleep

import cantools
import can
import rclpy
import time
from geometry_msgs.msg import Polygon
from radar_msgs.msg import RadarTrack, RadarTrackArray
from rclpy.node import Node

# assuming a 20hz update for the radar, retain memory of a target within a radar track for 0.5 second
# (this would increment for 5 ticks, then decrement for 5 ticks, totalling 10 ticks / 20 hz = 0.5 seconds)
RADAR_VALID_MAX = 5


class ECU:
    CAM = 0  # camera
    DSU = 1  # driving support unit
    APGS = 2  # advanced parking guidance system


class CAR:
    PRIUS = 0
    LEXUS_RXH = 1
    RAV4 = 2
    RAV4H = 3
    COROLLA = 4


# XXX: The Corolla is the targeted car for now
STATIC_MSGS = [
    (0x141, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 1, 2, '\x00\x00\x00\x46'),
    (0x128, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 1, 3,
     '\xf4\x01\x90\x83\x00\x37'),
    (0x283, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0, 3,
     '\x00\x00\x00\x00\x00\x00\x8c'),
    (0x344, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0, 5,
     '\x00\x00\x01\x00\x00\x00\x00\x50'),
    (0x160, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 1, 7,
     '\x00\x00\x08\x12\x01\x31\x9c\x51'),
    (0x161, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 1, 7,
     '\x00\x1e\x00\x00\x00\x80\x07'),
    (0x365, ECU.DSU, (CAR.RAV4, CAR.COROLLA), 0, 20, '\x00\x00\x00\x80\xfc\x00\x08'),
    (0x366, ECU.DSU, (CAR.RAV4, CAR.COROLLA), 0, 20, '\x00\x72\x07\xff\x09\xfe\x00'),
    (0x4CB, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0, 100,
     '\x0c\x00\x00\x00\x00\x00\x00\x00'),
]


class RadarMessage:
    def __init__(self):
        self.track_id = 0
        self.counter = 0
        self.lat_dist = 0.0
        self.lng_dist = 0.0
        self.new_track = True
        self.rel_speed = 0.0
        self.valid_count = 0
        self.valid = False
        self.filt_lng_dist = 0.0
        self.filt_rel_speed = 0.0
        self.can_timestamp = 0.0


class CanMessage:
    def __init__(self, msg):
        self.interface = "RADAR"
        self.id = msg.arbitration_id
        self.can_timestamp = msg.timestamp
        self.data = list(msg.data)
        self.is_extended = msg.is_extended_id
        self.is_error = msg.is_error_frame

    def __repr__(self):
        return f"""
id: {self.id} 
timestamp: {self.can_timestamp} 
data: {self.data}
extended: {self.is_extended}
error: {self.is_error}
"""


class ToyotaRadarController(Node, can.Listener):
    RADAR_TRACK_ID_START = 528
    RADAR_TRACK_ID_RANGE = 16
    RADAR_TRACK_ID_END = RADAR_TRACK_ID_START + RADAR_TRACK_ID_RANGE - 1  # 543
    RADAR_TRACK_ACCEL_ID_START = RADAR_TRACK_ID_END + 1  # 544
    RADAR_TRACK_ACCEL_ID_END = RADAR_TRACK_ACCEL_ID_START + RADAR_TRACK_ID_RANGE - 1  # 559

    """
    This radar controller is hardcoded to work only with the Toyota Corolla/Rav4/Camry 2017 Denso unit
    """

    def __init__(self):
        super().__init__('radar')
        self.tracks_pub = self.create_publisher(RadarTrackArray, 'radar_tracks', 10)
        self.adas_db = cantools.db.load_file('data/toyota_prius_2017_adas.dbc')

        self.radar_pub = self.create_publisher(RadarTrackArray, 'radar_tracks', 10)

        channel = self.declare_parameter("channel", "/dev/tty.usbmodem141301").value

        self.can_bus = can.interface.Bus(bustype="slcan", channel=channel, bitrate=500000)
        self.can_notifier = can.Notifier(self.can_bus, [self], timeout=0.0001)

        # This triggers self.power_on_radar() @ 100hz.
        # Based on OpenPilot, this is the base update rate required for delivering the CAN messages defined in STATIC_MSGS
        self.rate = 1.0 / 100.0
        # self.power_on_timer = self.create_timer(self.rate, self.power_on_radar)
        # self.rate = rclpy.Rate(self.power_on_timer)

        self.radar_is_on = False
        self.frame = 0
        self.start_time = time.clock_gettime(time.CLOCK_MONOTONIC_RAW)
        self.ticks = 0

        self.current_radar_counter = 0
        self.current_track_ids = {}
        self.current_radar_accels = []
        self.cache_radar_tracks = {}

        # only need to create these once, they are not recreated in the message loop
        for i in range(0, self.RADAR_TRACK_ID_RANGE):
            """
            geometry_msgs / Polygon
            track_shape  # The shape and position of the detection. This polygon
            # encompasses a 2D plane which approximates the size and
            # shape of the detection based on the distance from the
            # radar, the detection angle, the width of all detections
            # grouped into this track, and the height of the radar's
            # vertical field of view at the detection distance.

            geometry_msgs / Vector3
            linear_velocity  # Only the x and y components are valid.
            geometry_msgs / Vector3
            linear_acceleration  # Only the x component is valid.
            """
            track = RadarMessage()
            track.track_id = i

            self.cache_radar_tracks[i] = track

        self.reset_tracks()
        self.loop()

    def reset_tracks(self):
        self.current_track_ids = {}
        self.current_radar_accels = []

    def on_message_received(self, msg):
        # print("MESSAGE: ", msg)
        outmsg = CanMessage(msg)
        # print("OUTMESSAGE: ", outmsg)
        self.on_can_message(outmsg)


    def loop(self):
        # rate = rclpy
        # ct = time.clock_gettime(time.CLOCK_MONOTONIC_RAW)
        # ticks = (ct - self.start_time) / self.rate
        while rclpy.ok():
            # if int(ticks) != self.ticks:
            self.power_on_radar()
            # self.rate.sleep()


    def power_on_radar(self):
        print("SENT PWR MESSAGE")
        for (addr, ecu, cars, bus, fr_step, vl) in STATIC_MSGS:
            if self.frame % fr_step == 0:
                if addr in (0x489, 0x48a) and bus == 0:
                    # add counter for those 2 messages (last 4 bits)
                    cnt = ((frame / 100) % 0xf) + 1
                    if addr == 0x48a:
                        # 0x48a has a 8 preceding the counter
                        cnt += 1 << 7
                    vl += chr(cnt)
                tosend = bytearray()
                tosend.extend(map(ord, vl))
                message = can.Message(arbitration_id=addr, data=tosend, extended_id=False)
                self.can_bus.send(message)
        sleep(0.01)
        self.frame += 1.


    def on_can_message(self, can_msg):
        # print("ID: ", can_msg.id)
        # print("DATA: ", can_msg.data)
        if self.RADAR_TRACK_ID_START <= can_msg.id <= self.RADAR_TRACK_ACCEL_ID_END:
            #     #print("\n Message raw data:", can_msg.data)
            #     #print("\n Message bytearray", bytearray(can_msg.data))
            msg = self.adas_db.decode_message(can_msg.id, bytearray(can_msg.data))
            print("\n Decoded message: ", msg)
        #
        #     if self.current_radar_counter != msg["COUNTER"]:
        #         self.current_radar_counter = msg["COUNTER"]
        #         print("\n Decoded message: ", msg)

        #         current_radar_tracks = []
        #         current_radar_accels = self.current_radar_accels
        #
        #         # decrease the valid count for all of the tracks that were missing
        #         for k, track in self.cache_radar_tracks.items():
        #             if k not in self.current_track_ids:
        #                 valid_count = track.valid_count = max(0, track.valid_count - 1)
        #                 if valid_count > 0:
        #                     # add track to list from cache, with a invalid flag, but no accel for simplicity's sake
        #                     track.counter = self.current_radar_counter
        #                     track.valid = False
        #
        #             if (k in self.current_track_ids and self.current_track_ids[k]) or track.valid_count > 0:
        #                 current_radar_tracks.append(track)
        #
        #         # new update, send this track list
        #         if len(current_radar_tracks) > 0 or len(current_radar_accels) > 0:
        #             radar_tracks_msg = RadarTrackArray()
        #             print("current radar tracks", current_radar_tracks)
        #             # radar_tracks_msg.stamp = util.time_stamp_opencaret(can_msg.can_timestamp)
        #             # radar_tracks_msg.radar_tracks = current_radar_tracks
        #             # radar_tracks_msg.radar_accels = current_radar_accels
        #             # self.radar_pub.publish(radar_tracks_msg)
        #
        #         print("Reset tracks")
        #         print(self.current_radar_counter)
        #         self.reset_tracks()
        #         self.current_radar_counter = msg["COUNTER"]
        #
        #     if self.RADAR_TRACK_ID_START <= can_msg.id <= self.RADAR_TRACK_ID_END:
        #
        #         track_id = can_msg.id - self.RADAR_TRACK_ID_START
        #         track = self.cache_radar_tracks[track_id]
        #
        #         if msg['LONG_DIST'] >= 255 or msg['NEW_TRACK']:
        #             track.valid_count = 0  # reset counter
        #
        #         curr_valid_count = track.valid_count
        #         curr_valid_count += (1 if msg["VALID"] and msg['LONG_DIST'] < 255 else -1)
        #         curr_valid_count = min(RADAR_VALID_MAX, max(0, curr_valid_count))
        #
        #         assert not (msg["VALID"] and msg['LONG_DIST']) or curr_valid_count > 0, print(msg, curr_valid_count)
        #
        #         track.counter = msg["COUNTER"]
        #         track.lat_dist = msg["LAT_DIST"]
        #         track.lng_dist = msg["LONG_DIST"]
        #         track.rel_speed = msg["REL_SPEED"]
        #         track.new_track = bool(msg["NEW_TRACK"])
        #         track.valid_count = curr_valid_count
        #         track.valid = bool(msg["VALID"])
        #
        #         self.current_track_ids[track_id] = msg['LONG_DIST'] <= 255
        #
        #         if msg["VALID"] == 1:
        #             self.radar_is_on = True
        #
        #     elif self.RADAR_TRACK_ACCEL_ID_START <= can_msg.id <= self.RADAR_TRACK_ACCEL_ID_END:
        #         track_id = can_msg.id - self.RADAR_TRACK_ACCEL_ID_START
        #         #accel = RadarTrack(track_id=track_id,
        #         #                  linear_acceleration=float(msg["REL_ACCEL"]))
        #         print("Rel accel", float(msg["REL_ACCEL"]))
        #         #self.current_radar_accels.append(accel)


def main():
    rclpy.init()
    radar = ToyotaRadarController()
    # while rclpy.ok():
    # radar.on_loop()
    # rclpy.spin_once(radar, timeout_sec=radar.rate)
    # rclpy.spin(radar)

    #rclpy.spin(radar)

    radar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
