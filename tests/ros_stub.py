"""Inert ROS transport for testing the real production Python modules."""
import copy
import sys
import types
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
NOW = 100.0
PARAMS = yaml.safe_load((ROOT/'src/risabot_automode/config/params.yaml').read_text())

class Message:
    def __init__(self, data=None, **kw):
        self.data = data
        self.__dict__.update(kw)

class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0., y=0., z=0.)
        self.angular = types.SimpleNamespace(x=0., y=0., z=0.)

class Odometry:
    def __init__(self):
        self.twist = types.SimpleNamespace(twist=Twist())
        self.header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=100, nanosec=0), frame_id='')
        self.pose = types.SimpleNamespace(pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.,y=0.,z=0.),
            orientation=types.SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))

class Stamp:
    def __init__(self, seconds): self.nanoseconds = int(seconds*1e9)
    def __sub__(self, other): return Stamp((self.nanoseconds-other.nanoseconds)/1e9)
    def to_msg(self): return types.SimpleNamespace(sec=int(self.nanoseconds/1e9), nanosec=0)

class Pub:
    def __init__(self): self.messages=[]
    def publish(self, msg): self.messages.append(copy.deepcopy(msg))

class Parameter:
    Type=types.SimpleNamespace(INTEGER=2, DOUBLE=3)
    def __init__(self, name, kind=None, value=None): self.name, self.value=name,value

class Node:
    def __init__(self, name): self.name, self.params, self.subs=name,{},{}
    def declare_parameter(self, name, value):
        self.params[name]=PARAMS.get(self.name,{}).get('ros__parameters',{}).get(name,value)
    def get_parameter(self,name): return types.SimpleNamespace(value=self.params[name])
    def add_on_set_parameters_callback(self,cb): self.param_cb=cb
    def set_parameters(self, params):
        result = self.param_cb(params) if hasattr(self,'param_cb') else Message(successful=True)
        if result.successful:
            self.params.update({p.name:p.value for p in params})
        return [result]
    def create_publisher(self,*args,**kw): return Pub()
    def create_subscription(self,kind,topic,cb,*args): self.subs[topic]=cb
    def create_timer(self,*args): return types.SimpleNamespace(cancel=lambda:None)
    def destroy_timer(self,*args): pass
    def get_name(self): return self.name
    def get_clock(self): return types.SimpleNamespace(now=lambda:Stamp(NOW))
    def get_logger(self): return types.SimpleNamespace(**{k:lambda *a,**kw:None for k in ('info','warn','warning','debug','error')})

class Bot:
    def __init__(self): self.writes=[]
    def create_receive_threading(self): pass
    def set_auto_report_state(self,*a): pass
    def set_motor(self,*a): self.writes.append(('motor',a))
    def set_pwm_servo(self,*a): self.writes.append(('servo',a))

def module(name, **attrs):
    obj=types.ModuleType(name); obj.__dict__.update(attrs); sys.modules[name]=obj; return obj

def install():
    for package in ('risabot_automode','control_servo','obstacle_avoidance','obstacle_avoidance_camera','risabot_sim'):
        sys.path.insert(0,str(ROOT/'src'/package))
    qos=types.SimpleNamespace(SENSOR_DATA=types.SimpleNamespace(value=0))
    module('rclpy', Parameter=Parameter)
    module('rclpy.node',Node=Node); module('rclpy.qos',QoSPresetProfiles=qos)
    for name, attrs in {
        'std_msgs.msg':dict(Bool=Message,String=Message,Float32=Message),
        'geometry_msgs.msg':dict(Twist=Twist), 'nav_msgs.msg':dict(Odometry=Odometry),
        'sensor_msgs.msg':dict(Image=Message,LaserScan=Message,Joy=Message,Imu=Message),
        'rcl_interfaces.msg':dict(SetParametersResult=Message),
    }.items():
        module(name.split('.')[0]); module(name,**attrs)
    module('cv_bridge',CvBridge=lambda:None)
    module('Rosmaster_Lib',Rosmaster=Bot)
