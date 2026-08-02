from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class HandMsgDirection(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    FROM_HAND: _ClassVar[HandMsgDirection]
    TO_HAND: _ClassVar[HandMsgDirection]

class HandMainMsgType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    CONFIG: _ClassVar[HandMainMsgType]
    DATA: _ClassVar[HandMainMsgType]
    CMD: _ClassVar[HandMainMsgType]

class HandChipType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    ESP32_S3_MINI: _ClassVar[HandChipType]
    BQ27427: _ClassVar[HandChipType]
    KX132_1211: _ClassVar[HandChipType]
    VL53L1X: _ClassVar[HandChipType]
    CH101: _ClassVar[HandChipType]
    BOS1901: _ClassVar[HandChipType]
    BMI323: _ClassVar[HandChipType]
    TCA6408A: _ClassVar[HandChipType]

class HandChipInstance(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    ESP32_S3_MINI_MAIN: _ClassVar[HandChipInstance]
    BQ27427_BATTERY: _ClassVar[HandChipInstance]
    KX132_1211_SENSOR1: _ClassVar[HandChipInstance]
    KX132_1211_SENSOR2: _ClassVar[HandChipInstance]
    KX132_1211_SENSOR3: _ClassVar[HandChipInstance]
    KX132_1211_SENSOR4: _ClassVar[HandChipInstance]
    VL53L1X_SENSOR1: _ClassVar[HandChipInstance]
    VL53L1X_SENSOR2: _ClassVar[HandChipInstance]
    CH101_SENSOR1: _ClassVar[HandChipInstance]
    CH101_SENSOR2: _ClassVar[HandChipInstance]
    CH101_SENSOR3: _ClassVar[HandChipInstance]
    CH101_SENSOR4: _ClassVar[HandChipInstance]
    BOS1901_ACTUATOR1: _ClassVar[HandChipInstance]
    BOS1901_ACTUATOR2: _ClassVar[HandChipInstance]
    BOS1901_ACTUATOR3: _ClassVar[HandChipInstance]
    BOS1901_ACTUATOR4: _ClassVar[HandChipInstance]
    BMI323_IMU: _ClassVar[HandChipInstance]
    TCA6408A_CH101: _ClassVar[HandChipInstance]
    TCA6408A_OTHER: _ClassVar[HandChipInstance]

class HandDataType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    UINT8: _ClassVar[HandDataType]
    UINT16: _ClassVar[HandDataType]
    INT16: _ClassVar[HandDataType]
    INT32: _ClassVar[HandDataType]
    INT64: _ClassVar[HandDataType]
    FLOAT: _ClassVar[HandDataType]
    DOUBLE: _ClassVar[HandDataType]
    CH101_SIMPLE: _ClassVar[HandDataType]
    CH101_AMP: _ClassVar[HandDataType]
    CH101_IQ: _ClassVar[HandDataType]
FROM_HAND: HandMsgDirection
TO_HAND: HandMsgDirection
CONFIG: HandMainMsgType
DATA: HandMainMsgType
CMD: HandMainMsgType
ESP32_S3_MINI: HandChipType
BQ27427: HandChipType
KX132_1211: HandChipType
VL53L1X: HandChipType
CH101: HandChipType
BOS1901: HandChipType
BMI323: HandChipType
TCA6408A: HandChipType
ESP32_S3_MINI_MAIN: HandChipInstance
BQ27427_BATTERY: HandChipInstance
KX132_1211_SENSOR1: HandChipInstance
KX132_1211_SENSOR2: HandChipInstance
KX132_1211_SENSOR3: HandChipInstance
KX132_1211_SENSOR4: HandChipInstance
VL53L1X_SENSOR1: HandChipInstance
VL53L1X_SENSOR2: HandChipInstance
CH101_SENSOR1: HandChipInstance
CH101_SENSOR2: HandChipInstance
CH101_SENSOR3: HandChipInstance
CH101_SENSOR4: HandChipInstance
BOS1901_ACTUATOR1: HandChipInstance
BOS1901_ACTUATOR2: HandChipInstance
BOS1901_ACTUATOR3: HandChipInstance
BOS1901_ACTUATOR4: HandChipInstance
BMI323_IMU: HandChipInstance
TCA6408A_CH101: HandChipInstance
TCA6408A_OTHER: HandChipInstance
UINT8: HandDataType
UINT16: HandDataType
INT16: HandDataType
INT32: HandDataType
INT64: HandDataType
FLOAT: HandDataType
DOUBLE: HandDataType
CH101_SIMPLE: HandDataType
CH101_AMP: HandDataType
CH101_IQ: HandDataType

class HandDataMsg(_message.Message):
    __slots__ = ("source", "data_type", "data_count", "timestamp", "timestamps", "data")
    SOURCE_FIELD_NUMBER: _ClassVar[int]
    DATA_TYPE_FIELD_NUMBER: _ClassVar[int]
    DATA_COUNT_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMPS_FIELD_NUMBER: _ClassVar[int]
    DATA_FIELD_NUMBER: _ClassVar[int]
    source: HandChipInstance
    data_type: HandDataType
    data_count: int
    timestamp: int
    timestamps: _containers.RepeatedScalarFieldContainer[int]
    data: bytes
    def __init__(self, source: _Optional[_Union[HandChipInstance, str]] = ..., data_type: _Optional[_Union[HandDataType, str]] = ..., data_count: _Optional[int] = ..., timestamp: _Optional[int] = ..., timestamps: _Optional[_Iterable[int]] = ..., data: _Optional[bytes] = ...) -> None: ...

class HandDataMsgSimple(_message.Message):
    __slots__ = ("source", "data_type", "data_count", "data")
    SOURCE_FIELD_NUMBER: _ClassVar[int]
    DATA_TYPE_FIELD_NUMBER: _ClassVar[int]
    DATA_COUNT_FIELD_NUMBER: _ClassVar[int]
    DATA_FIELD_NUMBER: _ClassVar[int]
    source: HandChipInstance
    data_type: HandDataType
    data_count: int
    data: bytes
    def __init__(self, source: _Optional[_Union[HandChipInstance, str]] = ..., data_type: _Optional[_Union[HandDataType, str]] = ..., data_count: _Optional[int] = ..., data: _Optional[bytes] = ...) -> None: ...

class HandDataMsgAmp(_message.Message):
    __slots__ = ("source", "data_type", "data_count", "data")
    SOURCE_FIELD_NUMBER: _ClassVar[int]
    DATA_TYPE_FIELD_NUMBER: _ClassVar[int]
    DATA_COUNT_FIELD_NUMBER: _ClassVar[int]
    DATA_FIELD_NUMBER: _ClassVar[int]
    source: HandChipInstance
    data_type: HandDataType
    data_count: int
    data: bytes
    def __init__(self, source: _Optional[_Union[HandChipInstance, str]] = ..., data_type: _Optional[_Union[HandDataType, str]] = ..., data_count: _Optional[int] = ..., data: _Optional[bytes] = ...) -> None: ...

class HandDataMsgIq(_message.Message):
    __slots__ = ("source", "data_type", "data_count", "data")
    SOURCE_FIELD_NUMBER: _ClassVar[int]
    DATA_TYPE_FIELD_NUMBER: _ClassVar[int]
    DATA_COUNT_FIELD_NUMBER: _ClassVar[int]
    DATA_FIELD_NUMBER: _ClassVar[int]
    source: HandChipInstance
    data_type: HandDataType
    data_count: int
    data: bytes
    def __init__(self, source: _Optional[_Union[HandChipInstance, str]] = ..., data_type: _Optional[_Union[HandDataType, str]] = ..., data_count: _Optional[int] = ..., data: _Optional[bytes] = ...) -> None: ...

class HandConfigMsg(_message.Message):
    __slots__ = ("target", "config")
    TARGET_FIELD_NUMBER: _ClassVar[int]
    CONFIG_FIELD_NUMBER: _ClassVar[int]
    target: _containers.RepeatedScalarFieldContainer[HandChipInstance]
    config: bytes
    def __init__(self, target: _Optional[_Iterable[_Union[HandChipInstance, str]]] = ..., config: _Optional[bytes] = ...) -> None: ...

class HandCmdMsg(_message.Message):
    __slots__ = ("target", "cmd")
    TARGET_FIELD_NUMBER: _ClassVar[int]
    CMD_FIELD_NUMBER: _ClassVar[int]
    target: HandChipInstance
    cmd: bytes
    def __init__(self, target: _Optional[_Union[HandChipInstance, str]] = ..., cmd: _Optional[bytes] = ...) -> None: ...

class HandDataWrapper(_message.Message):
    __slots__ = ("data_msgs", "data_msgs_simple", "data_msgs_amp", "data_msgs_iq")
    DATA_MSGS_FIELD_NUMBER: _ClassVar[int]
    DATA_MSGS_SIMPLE_FIELD_NUMBER: _ClassVar[int]
    DATA_MSGS_AMP_FIELD_NUMBER: _ClassVar[int]
    DATA_MSGS_IQ_FIELD_NUMBER: _ClassVar[int]
    data_msgs: _containers.RepeatedCompositeFieldContainer[HandDataMsg]
    data_msgs_simple: _containers.RepeatedCompositeFieldContainer[HandDataMsgSimple]
    data_msgs_amp: _containers.RepeatedCompositeFieldContainer[HandDataMsgAmp]
    data_msgs_iq: _containers.RepeatedCompositeFieldContainer[HandDataMsgIq]
    def __init__(self, data_msgs: _Optional[_Iterable[_Union[HandDataMsg, _Mapping]]] = ..., data_msgs_simple: _Optional[_Iterable[_Union[HandDataMsgSimple, _Mapping]]] = ..., data_msgs_amp: _Optional[_Iterable[_Union[HandDataMsgAmp, _Mapping]]] = ..., data_msgs_iq: _Optional[_Iterable[_Union[HandDataMsgIq, _Mapping]]] = ...) -> None: ...

class HandConfigWrapper(_message.Message):
    __slots__ = ("config_msgs",)
    CONFIG_MSGS_FIELD_NUMBER: _ClassVar[int]
    config_msgs: _containers.RepeatedCompositeFieldContainer[HandConfigMsg]
    def __init__(self, config_msgs: _Optional[_Iterable[_Union[HandConfigMsg, _Mapping]]] = ...) -> None: ...

class HandCmdWrapper(_message.Message):
    __slots__ = ("cmd_msgs",)
    CMD_MSGS_FIELD_NUMBER: _ClassVar[int]
    cmd_msgs: _containers.RepeatedCompositeFieldContainer[HandCmdMsg]
    def __init__(self, cmd_msgs: _Optional[_Iterable[_Union[HandCmdMsg, _Mapping]]] = ...) -> None: ...

class HandMsg(_message.Message):
    __slots__ = ("bytes_count", "direction", "msg_type", "chip_type", "data_wrapper", "config_wrapper", "cmd_wrapper")
    BYTES_COUNT_FIELD_NUMBER: _ClassVar[int]
    DIRECTION_FIELD_NUMBER: _ClassVar[int]
    MSG_TYPE_FIELD_NUMBER: _ClassVar[int]
    CHIP_TYPE_FIELD_NUMBER: _ClassVar[int]
    DATA_WRAPPER_FIELD_NUMBER: _ClassVar[int]
    CONFIG_WRAPPER_FIELD_NUMBER: _ClassVar[int]
    CMD_WRAPPER_FIELD_NUMBER: _ClassVar[int]
    bytes_count: int
    direction: HandMsgDirection
    msg_type: HandMainMsgType
    chip_type: HandChipType
    data_wrapper: HandDataWrapper
    config_wrapper: HandConfigWrapper
    cmd_wrapper: HandCmdWrapper
    def __init__(self, bytes_count: _Optional[int] = ..., direction: _Optional[_Union[HandMsgDirection, str]] = ..., msg_type: _Optional[_Union[HandMainMsgType, str]] = ..., chip_type: _Optional[_Union[HandChipType, str]] = ..., data_wrapper: _Optional[_Union[HandDataWrapper, _Mapping]] = ..., config_wrapper: _Optional[_Union[HandConfigWrapper, _Mapping]] = ..., cmd_wrapper: _Optional[_Union[HandCmdWrapper, _Mapping]] = ...) -> None: ...
