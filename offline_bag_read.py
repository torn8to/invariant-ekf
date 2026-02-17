from dataclasses import dataclass
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odom
from typing import TypeAlias

import os
from os import PathLike
from pathlib import Path
from typing import Union, Optional, Type
from dataclasses import dataclass

MsgTypeReturn: TypeAlias = Union[Odom, Imu, None]

class ImuOdomBagReader:
    def __init__(self, bag_folder:PathLike, imu_topic: str, odom_topic: str):
        self.imu_topic = imu_topic
        self.odom_topic = odom_topic
        self.sequential_reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=bag_folder, 
            mode="sqlite3"
        )
        converter_options = rosbag2_py.ConverterOptions()

        storage_filter = rosbag2_py.StorageFilter(topics_to_filter=[imu_topic, odom_topic])
        self.sequential_reader.open(storage_options, converter_options)
        self.sequential_reader.set_filter(storage_filter)
        self.type_map = {topic.name : topic.type for topic in self.sequential_reader.get_all_topics_and_types()}
        
    def iterate_next(self) -> tuple[MsgTypeReturn, float]:
        if self.sequential_reader.has_next():
            return (None, -1.0)
        topic, data, timestamp = self.sequential_reader.read_next()
        msg_type: Type[Imu, Odom] = get_message(self.type_map[topic])
        msg: MsgTypeReturn = deserialize_message(data, msg_type)
        return (msg, timestamp)

@dataclass
class StampedMsg:
    time:float
    msg: Union[Odom, Imu]
    
    def msg_type(self) -> Type[Odom, Imu]:
        return type(self.msg)

class TimeAlignedFeeder:
    def __init__(self, bag_reader:ImuOdomBagReader, imu_time_offset:float = 0.0):
        self.bag_reader: ImuOdomBagReader = bag_reader
        self.list: list[StampedMsg] = []
        self.odom_msg: Optional[StampedMsg] = None
        self.imu_time_offset = imu_time_offset

    def next_msg_in_lidar_time(self)->Optional[StampedMsg]:
        msg, time_stamp = self.bag_reader.iterate_next()
        if isinstance(msg, Imu):
            time_aligned  = time_stamp - self.imu_time_offset 
            return StampedMsg(time_aligned, msg)
        elif isinstance(msg, Odom):
            return StampedMsg(time_stamp, msg)
        else:
            return None
                






