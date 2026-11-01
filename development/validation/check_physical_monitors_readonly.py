"""Exercise physical monitor clients without creating any action client or goal."""
import json,os,time,math,hashlib
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from control_msgs.msg import DynamicJointState
from moveit_msgs.srv import GetPlanningScene
from rcl_interfaces.srv import GetParameters
from ur5e_block_bringup.physical_stack import PhysicalStackTask,JOINTS,scene_fingerprint
assert os.environ.get('ROS_DOMAIN_ID')=='0'
class MonitorProbe(Node):
    sensor_feedback=PhysicalStackTask.sensor_feedback
    sensor_fresh=PhysicalStackTask.sensor_fresh
    flight_queries=PhysicalStackTask.flight_queries
    prewarm_flight_queries=PhysicalStackTask.prewarm_flight_queries
    poll_flight_queries=PhysicalStackTask.poll_flight_queries
    destroy_flight_queries=PhysicalStackTask.destroy_flight_queries
    def __init__(self):
        super().__init__('physical_monitors_readonly')
        self.mock=False;self.stopped=False;self.stop_reason=None;self.monitor_clients={}
        self.values={};self.times={};self.sensor={};self.sensor_sequence=-1;self.sensor_received=-math.inf
        self.create_subscription(JointState,'/joint_states',self.feedback,qos_profile_sensor_data)
        self.create_subscription(DynamicJointState,'/gripper/dynamic_joint_states',self.sensor_feedback,qos_profile_sensor_data)
    def feedback(self,m):
        for name,q in zip(m.name,m.position):self.values[name]=q;self.times[name]=time.monotonic()
    def fresh(self):return all(time.monotonic()-self.times.get(j,-math.inf)<.5 for j in JOINTS)
    def check_stop(self):
        if self.stopped:raise RuntimeError(json.dumps(self.stop_reason,ensure_ascii=False))
    def call(self,kind,service,request):
        c=self.create_client(kind,service)
        if not c.wait_for_service(timeout_sec=10.):raise RuntimeError(service+' unavailable')
        f=c.call_async(request);rclpy.spin_until_future_complete(self,f,timeout_sec=10.)
        if not f.done() or f.result() is None:raise RuntimeError(service+' timeout')
        result=f.result();self.destroy_client(c);return result
rclpy.init();n=MonitorProbe();report={'actual_goals_sent':False,'rounds':[]}
try:
    r=GetParameters.Request();r.names=['robot_description']
    n.model=n.call(GetParameters,'/robot_state_publisher/get_parameters',r).values[0].string_value
    r=GetParameters.Request();r.names=['robot_description','robot_description_semantic']
    values=n.call(GetParameters,'/move_group/get_parameters',r).values
    assert values[0].string_value==n.model
    n.semantic=values[1].string_value
    r=GetPlanningScene.Request();r.components.components=1023
    expected=scene_fingerprint(n.call(GetPlanningScene,'/get_planning_scene',r).scene)
    began=time.monotonic()
    while not(n.fresh() and n.sensor_fresh()):
        assert time.monotonic()-began<10.,'Fresh actual feedback unavailable'
        rclpy.spin_once(n,timeout_sec=.02)
    for index in range(3):
        queries=n.flight_queries({},expected);started=time.monotonic()
        n.prewarm_flight_queries(queries);warm=time.monotonic()-started
        latencies=[];end=time.monotonic()+4.
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.02)
            assert n.fresh() and n.sensor_fresh(),'Feedback became stale'
            for q in queries:
                if q['future'] is not None and q['future'].done():
                    latencies.append({'service':q['service'],'latency':time.monotonic()-q['sent']})
            n.poll_flight_queries(queries);n.check_stop()
        assert all(q['verified'] for q in queries)
        n.destroy_flight_queries(queries)
        report['rounds'].append({'index':index+1,'warmup_seconds':warm,'client_count':len(n.monitor_clients),'responses':len(latencies),'max_response_seconds':max((v['latency']for v in latencies),default=0.)})
    report.update(result='passed',model_sha256=hashlib.sha256(n.model.encode()).hexdigest(),scene_hash=expected,stop_reason=n.stop_reason)
finally:
    if 'result'not in report:report.update(result='failed',stop_reason=n.stop_reason)
    Path('/home/ksn/ur5e_ws/validation/physical_monitors_actual_readonly.json').write_text(json.dumps(report,indent=2)+'\n')
    n.destroy_node();rclpy.shutdown()
print(json.dumps(report,ensure_ascii=False))
