#!/usr/bin/python2.4
#
# Martin Galpin (martin@66laps.com)
#
# Copyright (C) 2010 66laps Limited. All rights reserved.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 3
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301,
# USA.

'''
A library that provides a python interface to the Live For Speed RAF format. 

For in-depth details on RAF (including the data types and offsets used 
in this file) see: http://www.lfs.net/?page=RAF

The data types referenced in the RAF specification are mapped as follows:

LFS Type | Bytes | C Type         | Python type
---------------------------------------------------
char     | 1     | signed char    | integer (c)
byte     | 1     | unsigned char  | integer (B)
word     | 2     | unsigned short | integer (H)
int      | 4     | int            | integer (I)
short    | 2     | short          | integer (h)
float*   | 4     | float          | float (f)

* little-endian byte order
'''

__author__  = 'Martin Galpin'
__contact__ = 'martin@66laps.com'
__version__ = '0.1'
__license__ = 'GNU General Public License'

import struct, math

class StaticWheelInfo(object):
  '''This class represents static wheel information.'''
  def __init__(self, **kwargs):
    self.x = None                # relative to reference point
    self.y = None                # relative to reference point
    self.z = None                # relative to reference point
    self.radius = None           # unloaded
    self.width = None            # at widest point
    self.maximum_deflect = None  # suspension travel
    self.tyre_type = None        # see documentation
    self.spring_constant = None  # N/m
    self.damping_c = None        # Ns/m
    self.damping_r = None        # Ns/m
    self.max_brake_torque = None # Nm
    self.__dict__.update(kwargs)

  @staticmethod
  def from_file(f):
    '''
    Reads a block of static wheel info (128 bytes) from file f.
    '''
    buf = f.read((6 * FLOAT) + (8 * BYTE) + (4 * FLOAT))
    
    wheel = StaticWheelInfo()
    wheel.x, wheel.y, wheel.z, wheel.radius, wheel.width, \
    wheel.maximum_deflect, tyre_type, wheel.spring_constant, \
    wheel.damping_c, wheel.damping_r, wheel.max_brake_torque \
            = struct.unpack('<ffffffxxxxxBxxffff', buf)

    # read to the end of this block
    f.read(80 * BYTE)
    
    return wheel
    
  def __str__(self):
    '''
    Returns:
      A string representation of this lfs.StaticWheelInfo instance.
    '''
    return self.__dict__.__str__()
    
class DynamicWheelInfo(object):
  '''This class represents dynamic wheel information.'''
  
  def __init__(self, **kwargs):
    self.suspension_deflect = None # compression from unloaded
    self.steer = None              # including Ackermann and toe
    self.x_force = None            # force right
    self.y_force = None            # force forward
    self.vertical_load = None      # perpendicular to surface
    self.angular_velocity = None   # radians/s
    self.lean = None               # releative to road (radians/s) viewed from rear
    self.air_temp = None           # degrees C
    self.slip_fraction = None      # 0 - 255
    self.__dict__.update(kwargs)
  
  @staticmethod
  def from_file(f):
    '''Read a block of dynamic wheel info (32-bytes) from file f.'''
    buf = f.read((7 * FLOAT) + (4 * BYTE))
    
    wheel = DynamicWheelInfo()
    wheel.suspension_deflect, wheel.steer, wheel.x_force, wheel.y_force, \
    wheel.vertical_load, wheel.angular_velocity, wheel.lean, wheel.air_temp, \
    wheel.slip_fraction = struct.unpack('<fffffffbbxx', buf)
    
    return wheel
    
  def __repr__(self):
    '''
    Returns:
      A string representation of this lfs.DynamicWheelInfo instance.
    '''
    return self.__dict__.__str__()
    
class DataBlock(object):
  '''This class represents a single sampled datablock.'''

  def __init__(self, **kwargs):
    self.throttle = None       # throttle (0 to 1)          
    self.brake = None          # brake (0 to 1)
    self.input_steer = None    # input steer (radians)
    self.clutch = None         # clutch (0 to 1)
    self.handbrake = None      # handbrake (0 to 1)
    self.gear = None           # gear (0=R 1=N 2=first gear)
    self.lateral_g = None      # lateral G -120 to 120 = -6 to 6G
    self.forward_g = None      # forward G -120 to 120 = -6 to 6G
    self.upwards_g = None      # upwards G -120 to 120 = -6 to 6G
    self.speed = None          # m/s
    self.car_distance = None   # m - travelled by car
    self.position_x = None     # map X (1m = 65536)
    self.position_y = None     # map Y (1m = 65536)
    self.position_z = None     # map Z (1m = 65536)
    self.engine_speed = None   # radians/s
    self.index_distance = None # m - track ruler measurement
    self.rx = None             # x of right-vector
    self.ry = None             # y of right-vector
    self.rz = None             # z of right-vector
    self.fx = None             # x of forward-vector
    self.fy = None             # y of forward-vector
    self.fz = None             # z of forward-vector
    self.heading = None        # anti-clockwise from above (calculated)
    self.wheels = []           # an instance of DynamicWheelInfo for each wheel
    self.__dict__.update(kwargs)
  
  @staticmethod
  def from_file(f):
    '''Reads a data block (128 bytes) from the current offset of file f.'''
    block = DataBlock()

    buf = f.read((9 * FLOAT) + BYTE + (3 * CHAR) + (3 * INT) + (6 * SHORT))    
    block.throttle, block.brake, block.input_steer, block.clutch, \
    block.handbrake, block.gear, block.lateral_g, block.forward_g, \
    block.upwards_g, block.speed, block.car_distance, block.position_x, \
    block.position_y, block.position_z, block.engine_speed, \
    block.index_distance, block.rx, block.ry, block.rz, block.fx, block.fy, block.fz \
      = struct.unpack('<fffffbbbbffiiiffhhhhhh', buf)
      
    # calculate the heading of the vehicle at this point for completeness
    b = block.fx / 32767.0
    e = block.fy / 32767.0
    block.heading = math.atan2(-b, e)
      
    return block
    
  def __repr__(self):
    '''Returns:
      A string representation of this lfs.DataBlock instance.
    '''
    return self.__dict__.__str__()

class Replay(object):
  '''
  This class represents a LFS Replay Analyzer Format Version 2 file.
  
  Usage:
    import lfs
    raf = lfs.Replay('hotlap.raf')
  '''
  def __init__(self, path=None, **kwargs):
    '''Creates an instance of lfs.Replay. 
    
    This constructor basically provides an alternative to calling 
    lfs.Replay.from_file() with an open instance of file.
    
    Args:
      path
        The path to the Live For Speed RAF file to open.
    '''  
    self.raf_version = None              # Live For Speed RAF version
    self.lfs_version = None              # Live For Speed version  
    self.update_interval = None          # ms
    self.short_track_name = None         # e.g. BL2R
    self.track_ruler_length = None       # total index distance
    self.player = None                   # player name
    self.car = None                      # car name
    self.track = None                    # track name
    self.config = None                   # track configuration
    self.weather = None                  # weather description
    self.player_flags = None             # driver aids (see docs)
    self.num_wheels = None               # number of wheels (hopefully four...)
    self.hlvc_legal = None               # hotlap legal: Unknown (None), True, False
    self.num_splits = None               # number of splits (includes lap time)
    self.splits = []                     # splits (always four)
    self.mass = None                     # mass (kg)
    self.sprung_mass = None              # sprung_mass (kg)
    self.antiroll_rear = None            # n/M
    self.antiroll_front = None           # n/M
    self.final_drive = None              # final drive ratio
    self.num_gears = None                # number of forward gears
    self.gear_ratios = []                # gear ratios (forward)

    self.static_wheel_info = []          # instance of StaticWheelInfo for each wheel
    self.data = []                       # instance of DataBlock for each sample
    
    if path:
      with open(path, 'rb') as f:
        self.__dict__.update(Replay.from_file(f).__dict__)
    else:
      self.__dict__.update(**kwargs)
  
  @staticmethod
  def from_file(f):
    '''Reads an open file instance and returns and instance of lfs.Replay.'''
    replay = Replay()
    
    # we need the LFSRAF header
    buf = f.read(6 * CHAR)
    lfsraf = ''.join(struct.unpack(">cccccc", buf))
    if not lfsraf == 'LFSRAF':
      raise InputError("Live For Speed RAF header not found.") 

    # skip game version/revision
    f.read(2 * BYTE)

    buf = f.read(2 * BYTE)
    replay.raf_version, replay.update_interval = struct.unpack('>BB', buf)
    if replay.raf_version > 2:
      raise InputError("Unknown RAF version: %d" % replay.raf_version)

    # skip empty blocks
    f.read(2 * BYTE)

    # header size, block size, wheel block size, wheel block offset, num blocks
    buf = f.read(4 * WORD + INT)
    header_size, block_size, wheel_block_size, \
      wheel_block_offset, num_blocks = struct.unpack('<HHHHI', buf)

    buf = f.read(4 * CHAR)
    replay.short_track_name = ''.join(struct.unpack('>cccc', buf))
    replay.short_track_name = \
      replay.short_track_name[:replay.short_track_name.find('\x00')]

    buf = f.read(FLOAT)
    replay.track_ruler_length = struct.unpack('<f', buf)[0]

    # driver, track, weather and LFS version
    buf = f.read(32 * CHAR)
    replay.player = ''.join(struct.unpack('>cccccccccccccccccccccccccccccccc', buf))
    replay.player = replay.player[:replay.player.find('\x00')]

    buf = f.read(32 * CHAR)
    replay.car = ''.join(struct.unpack('>cccccccccccccccccccccccccccccccc', buf))
    replay.car = replay.car[:replay.car.find('\x00')]

    buf = f.read(32 * CHAR)
    replay.track = ''.join(struct.unpack('>cccccccccccccccccccccccccccccccc', buf))
    replay.track = replay.track[:replay.track.find('\x00')]

    buf = f.read(16 * CHAR)
    replay.config = ''.join(struct.unpack('>cccccccccccccccc', buf))
    replay.config = replay.config[:replay.config.find('\x00')]

    buf = f.read(16 * CHAR)
    replay.weather = ''.join(struct.unpack('>cccccccccccccccc', buf))
    replay.weather = replay.weather[:replay.weather.find('\x00')]

    buf = f.read(8 * CHAR)
    replay.lfs_version = ''.join(struct.unpack('>cccccccc', buf))
    replay.lfs_version = replay.lfs_version[:replay.lfs_version.find('\x00')]

    buf = f.read(4 * BYTE)
    replay.player_flags, replay.num_wheels, \
      hlvc_legal, replay.num_splits = struct.unpack('>BBBB', buf)
    
    # 0 = UNKNOWN, 1 = LEGAL, 2 = ILLEGAL
    if hlvc_legal > 0: replay.player_flags = True if hlvc_legal == 1 else False

    buf = f.read(4 * INT)
    replay.splits.extend(struct.unpack('<iiii', buf))
    replay.splits = replay.splits[:replay.num_splits]
    
    buf = f.read(5 * FLOAT)
    replay.mass, replay.sprung_mass, replay.antiroll_rear, \
      replay.antiroll_front, replay.final_drive = struct.unpack('<fffff', buf)

    buf = f.read(BYTE)
    replay.num_gears = struct.unpack('>B', buf)[0]

    # skip empty blocks
    f.read(3 * BYTE)
    
    buf = f.read(7 * FLOAT)
    replay.gear_ratios.append(struct.unpack('<fffffff', buf))

    # skip empty blocks
    f.read(272 * BYTE)
    
    # read static wheel info dynamically incase we only have three...
    for i in range(0, replay.num_wheels):
      replay.static_wheel_info.append(StaticWheelInfo.from_file(f))

    # read data blocks until the end of the file
    blocks = []
    for i in range(0, num_blocks):
      block = DataBlock.from_file(f)
      
      # for each wheel read a 32-bytes of dynamic wheel info
      for i in range(0, replay.num_wheels):
        block.wheels.append(DynamicWheelInfo.from_file(f))
      
      replay.data.append(block)
      
    return replay
  
  def __repr__(self):
    '''
    Returns:
      A string representation of this lfs.Replay instance.
    '''
    return '%s %s %s %s' % (self.player, self.car, self.track, self.config)

# define LFS data types making file seeking more readable 
CHAR  = 1
BYTE  = 1
WORD  = 2
INT   = 4
FLOAT = 4
SHORT = 2