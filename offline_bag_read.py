from dataclasses import dataclass
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from typing import TypeAlias

import os
from os import PathLike
from pathlib import Path
from typing import Union, Optional, Type
from dataclasses import dataclass

MsgTypeReturn: TypeAlias = Union[Odometry, Imu, None]


class ImuOdometryBagReader:
    def __init__(self, bag_folder:PathLike, imu_topic: str, odom_topic: str, storage_mode = "mcap"):
        self.imu_topic = imu_topic
        self.odom_topic = odom_topic
        self.sequential_reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=bag_folder,
                                                    storage_id=storage_mode)
        converter_options = rosbag2_py.ConverterOptions()

        storage_filter = rosbag2_py.StorageFilter(topics=[imu_topic, odom_topic]) 
        self.sequential_reader.open(storage_options, converter_options)
        self.sequential_reader.set_filter(storage_filter)
        self.type_map = {topic.name: topic.type for topic in self.sequential_reader.get_all_topics_and_types()}
        self._size = self.get_message_count(bag_folder, [imu_topic, odom_topic], storage_mode)
        self._current_msg_count = -1

    @property
    def size(self) -> int:
        return self._size

    @property
    def current_msg_count(self) -> int:
        return self._current_msg_count

    def iterate_next(self) -> tuple[MsgTypeReturn, float]:
        self._current_msg_count = self._current_msg_count + 1
        if not self.sequential_reader.has_next():
            return (None, -1.0)
        topic, data, timestamp = self.sequential_reader.read_next()
        msg_type: Type[Imu, Odometry] = get_message(self.type_map[topic])
        msg: MsgTypeReturn = deserialize_message(data, msg_type)
        return (msg, timestamp)

    def get_message_count(self, bag_path: str, topic_list: list[str], storage_mode: str) -> int:
        msg_count: int = 0
        info = rosbag2_py.Info()
        metadata = info.read_metadata(bag_path, storage_mode)
        for topic_ in metadata.topics_with_message_count:
            if topic_.topic_metadata.name in topic_list:
                msg_count = msg_count + topic_.message_count
        return msg_count

@dataclass(slots=True)
class StampedMsg:
    time: float
    msg: Union[Odometry, Imu]

    @property
    def msg_type(self) -> Type[Odometry | Imu]:
        return type(self.msg)


class TimeAlignedBagFeeder:
    def __init__(self, bag_reader: ImuOdometryBagReader, imu_time_offset: float = 0.0):
        self.bag_reader: ImuOdometryBagReader = bag_reader
        self.odom_msg: Optional[StampedMsg] = None
        self.imu_time_offset = imu_time_offset

    @property
    def size(self):
        return self.bag_reader.size

    @property
    def current_msg_count(self):
        return self.bag_reader.current_msg_count

    def next_msg_in_lidar_time(self) -> Optional[StampedMsg]:
        msg, timestamp = self.bag_reader.iterate_next()
        if isinstance(msg, Imu):
            time_aligned = timestamp - self.imu_time_offset
            return StampedMsg(time_aligned, msg)
        elif isinstance(msg, Odometry):
            return StampedMsg(timestamp, msg)
        else:
            return None

    def __len__(self):
        return self.size

    def __iter__(self):
        while (msg:= self.next_msg_in_lidar_time()) is not None:
            yield msg
