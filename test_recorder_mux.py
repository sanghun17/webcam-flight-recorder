"""Recorder registration regressions; never connect to the live ROS master."""
import types
import unittest
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time
from unittest.mock import Mock, patch
import xmlrpc.client

import recorder_mux as mux


class RegistrationTest(unittest.TestCase):
    def setUp(self):
        self.master = Mock()
        self.master.getPid.return_value = (1, '', 85)
        self.master.lookupNode.return_value = (1, '', 'http://ml:123/')
        self.master.lookupService.return_value = (1, '', 'rosrpc://ml:456')
        self.services = [types.SimpleNamespace(resolved_name='/recorder/start', uri='rosrpc://ml:456'),
                         types.SimpleNamespace(resolved_name='/recorder/stop', uri='rosrpc://ml:456')]

    def problem(self):
        return mux._registration_problem(self.master, '/recorder', 'http://ml:123/', self.services)

    def test_live_node_and_both_services(self):
        self.assertIsNone(self.problem())
        self.assertEqual(self.master.lookupService.call_count, 2)

    def test_missing_node_with_reused_pid(self):
        self.master.lookupNode.return_value = (-1, 'unknown node', '')
        self.assertIn('node /recorder', self.problem())

    def test_different_node_incarnation(self):
        self.master.lookupNode.return_value = (1, '', 'http://ml:999/')
        self.assertIn('node /recorder', self.problem())

    def test_services_missing_even_when_node_exists(self):
        for missing in ['/recorder/start', '/recorder/stop']:
            with self.subTest(missing=missing):
                self.master.lookupService.side_effect = lambda name: (-1, '', '') if name == missing else (1, '', 'rosrpc://ml:456')
                self.assertIn('service '+missing, self.problem())

    def test_stale_service_address(self):
        self.master.lookupService.return_value = (1, '', 'rosrpc://ml:999')
        self.assertIn('service /recorder/start', self.problem())

    def test_missing_registration_exits_for_supervisor_restart(self):
        self.master.lookupNode.return_value = (-1, 'unknown node', '')
        with patch.object(mux.rospy, 'get_master', return_value=self.master), \
             patch.object(mux.rospy, 'get_name', return_value='/recorder'), \
             patch.object(mux.rospy, 'get_node_uri', return_value='http://ml:123/'), \
             patch.object(mux.rospy, 'is_shutdown', return_value=False), \
             patch.object(mux.rospy, 'signal_shutdown') as shutdown, \
             patch.object(mux.rospy, 'logwarn'), \
             patch.object(mux.time, 'sleep'), \
             patch.object(mux.os, '_exit', side_effect=SystemExit(1)):
            with self.assertRaises(SystemExit):
                mux._master_watchdog(self.services)
            shutdown.assert_called_once_with('ROS registration lost')

    def test_unreachable_master_is_retried_without_exit(self):
        self.master.getPid.side_effect = OSError('unreachable')
        with patch.object(mux.rospy, 'get_master', return_value=self.master), \
             patch.object(mux.rospy, 'get_name', return_value='/recorder'), \
             patch.object(mux.rospy, 'get_node_uri', return_value='http://ml:123/'), \
             patch.object(mux.rospy, 'is_shutdown', side_effect=[False, True]), \
             patch.object(mux.time, 'sleep') as sleep, \
             patch.object(mux.os, '_exit') as exit:
            mux._master_watchdog(self.services)
            exit.assert_not_called()
            sleep.assert_called_once_with(5)


@unittest.skipUnless(os.environ.get('WEBCAM_TEST_REAL_MASTER') == '1', 'isolated ROS integration opt-in')
class RealMasterTest(unittest.TestCase):
    def test_same_pid_restart_and_service_only_loss(self):
        """Real ROS API; private master, fake cameras, and no device access."""
        from std_srvs.srv import Trigger
        import rospy

        uri = 'http://127.0.0.1:11399'
        with socket.socket() as sock:
            self.assertNotEqual(sock.connect_ex(('127.0.0.1', 11399)), 0, 'private test port already in use')
        processes = []
        server = None
        camera_code = '''
import rospy
from std_srvs.srv import Trigger, TriggerResponse
rospy.init_node('test_cameras')
services=[rospy.Service('/recorder_'+cam+'/'+action,Trigger,lambda _:TriggerResponse(True,'test-only, no camera device')) for cam in ['cam1','cam2'] for action in ['start','stop']]
rospy.spin()
'''
        master_code = '''
import signal
from rosmaster.master import Master
from rosmaster.master_api import ROSMasterHandler
# Containers reuse PIDs. Keep the advertised master PID constant across real
# process replacements while closing all old sockets on termination.
ROSMasterHandler.getPid=lambda self,caller_id:(1,'test reused PID',85)
master=Master(port=11399);master.start()
signal.pause()
'''
        with tempfile.TemporaryDirectory(prefix='webcam-registration-test-') as tmp, \
             patch.dict(os.environ, ROS_MASTER_URI=uri, ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1', ROS_LOG_DIR=tmp):
            env = dict(os.environ)
            log = open(Path(tmp)/'children.log', 'w+')
            def spawn(args):
                proc = subprocess.Popen(['/usr/bin/python3', '-u', '-s']+args, env=env, stdout=log, stderr=log)
                processes.append(proc)
                return proc
            def wait_for(predicate, timeout=12):
                until = time.monotonic()+timeout
                while time.monotonic()<until:
                    try:
                        if predicate():
                            return
                    except (OSError, xmlrpc.client.Error):
                        pass
                    time.sleep(.1)
                log.flush(); log.seek(0)
                self.fail('isolated recovery timed out:\n'+log.read()[-5000:])
            def registered():
                return all(api.lookupService('/test', name)[0] == 1 for name in ['/recorder/start','/recorder/stop','/recorder_cam1/start','/recorder_cam1/stop','/recorder_cam2/start','/recorder_cam2/stop'])
            def trigger_both():
                # Service calls resolve only against this private master.
                for action in ['start','stop']:
                    response=rospy.ServiceProxy('/recorder/'+action,Trigger)()
                    self.assertTrue(response.success, response.message)
                    self.assertIn('/recorder_cam1',response.message)
                    self.assertIn('/recorder_cam2',response.message)
            try:
                api=xmlrpc.client.ServerProxy(uri)
                server=spawn(['-c',master_code])
                wait_for(lambda:api.getPid('/test')[0]==1)
                pid=api.getPid('/test')[2]
                camera=spawn(['-c',camera_code])
                process=spawn([str(Path(mux.__file__).resolve())])
                wait_for(registered);trigger_both()

                server.terminate();server.wait(timeout=5)
                server=spawn(['-c',master_code])
                api=xmlrpc.client.ServerProxy(uri)
                wait_for(lambda:api.getPid('/test')[0]==1)
                self.assertEqual(api.getPid('/test')[2],pid)
                wait_for(lambda:process.poll() is not None)
                # rospy.spin can return during the watchdog's shutdown grace;
                # Restart=always handles both a clean exit and os._exit(1).
                self.assertIn(process.returncode,(0,1))
                camera.terminate();camera.wait(timeout=5)
                camera=spawn(['-c',camera_code])
                process=spawn([str(Path(mux.__file__).resolve())])
                wait_for(registered);trigger_both()

                node_uri=api.lookupNode('/test','/recorder')[2]
                service_uri=api.lookupService('/test','/recorder/start')[2]
                self.assertEqual(api.unregisterService('/recorder','/recorder/start',service_uri)[0],1)
                self.assertEqual(api.lookupNode('/test','/recorder')[2],node_uri)
                wait_for(lambda:process.poll() is not None)
                self.assertIn(process.returncode,(0,1))
                process=spawn([str(Path(mux.__file__).resolve())])
                wait_for(registered);trigger_both()
            finally:
                for proc in processes:
                    if proc.poll() is None:
                        proc.terminate()
                        try:
                            proc.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            proc.kill();proc.wait(timeout=5)
                log.close()


if __name__ == '__main__':
    unittest.main()
