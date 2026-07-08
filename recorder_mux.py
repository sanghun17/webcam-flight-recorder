#!/usr/bin/env python3
"""
recorder_mux — fan a single /recorder/{start,stop} onto every per-camera node.

The onboard flight-safety recorder (on the Jetson) triggers webcam recording over
ROS by calling std_srvs/Trigger services /recorder/start (on ARM) and
/recorder/stop (on DISARM). With one camera the recorder node owned those names
directly; with several cameras each node lives in its own namespace
(/recorder_cam1, /recorder_cam2, ...). This tiny node re-advertises the original
/recorder/start and /recorder/stop and forwards each call to every camera, so the
onboard side needs NO change.

start: stamp one shared session stem (a param the cameras read, default
       /recorder/session_stem) so every camera writes flight_<stamp>_<tag>, then
       trigger each camera's ~start.
stop:  trigger each camera's ~stop and combine the replies. The onboard parser
       grabs the FIRST "<path>.mp4" in the reply to rename its bag, so the first
       camera in ~cameras (cam1) keys the bag / overlay folder.

Params (~private):
  ~cameras            list of camera namespaces (default [/recorder_cam1, /recorder_cam2])
  ~session_stem_param global param the cameras read for a shared name
  ~timeout            per-camera wait_for_service timeout seconds (default 5)

No device access; stdlib + rospy + std_srvs only.
"""
import datetime
import os
import threading
import time

import rospy
from std_srvs.srv import Trigger, TriggerResponse


class RecorderMux(object):
    def __init__(self):
        self.cams = list(rospy.get_param("~cameras", ["/recorder_cam1", "/recorder_cam2"]))
        self.stem_param = rospy.get_param("~session_stem_param", "/recorder/session_stem")
        self.timeout = float(rospy.get_param("~timeout", 5.0))
        rospy.set_param(self.stem_param, "")  # start with no pinned name
        self._stem = ""                        # last session stem, for the stop reply
        rospy.Service("~start", Trigger, self._start)
        rospy.Service("~stop", Trigger, self._stop)
        rospy.loginfo("[recorder_mux] fan-out ready: /recorder/start|stop -> %s",
                      ", ".join(self.cams))

    def _start(self, _req):
        # Pin one stem so every camera names its file flight_<stamp>_<tag> identically.
        stem = "flight_" + datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self._stem = stem
        rospy.set_param(self.stem_param, stem)
        results = self._fanout("start")
        rospy.set_param(self.stem_param, "")  # don't leak the stem into a later manual start
        parts = ["%s: %s" % (c, (r.message if r is not None else "unavailable"))
                 for c, r in zip(self.cams, results)]
        return TriggerResponse(self._all_ok(results), "session %s -> %s" % (stem, " | ".join(parts)))

    def _stop(self, _req):
        results = self._fanout("stop")
        ok = any(r is not None and r.success for r in results)
        cam_msg = " | ".join("%s: %s" % (c, (r.message if r is not None else "no response"))
                             for c, r in zip(self.cams, results))
        # The onboard recorder grabs the FIRST "<x>.mp4" in this reply and renames its
        # bag to that basename + syncs it to recordings/<basename>/. Emit the shared
        # stem first so the bag lands at recordings/<stem>/ — the flight folder holding
        # cam1/ and cam2/ — instead of inside one camera's subfolder.
        lead = ("%s.mp4 | " % self._stem) if self._stem else ""
        return TriggerResponse(ok, (lead + cam_msg) or "no cameras configured")

    def _fanout(self, action):
        results = [None] * len(self.cams)
        threads = []
        for i, ns in enumerate(self.cams):
            srv = ns.rstrip("/") + "/" + action
            t = threading.Thread(target=self._call_into, args=(results, i, srv, action))
            t.daemon = True
            t.start()
            threads.append(t)
        for t in threads:
            t.join(timeout=20.0)  # a camera stop can take ~10s (ffmpeg SIGINT finalize)
        return results

    def _call_into(self, results, i, srv, action):
        try:
            rospy.wait_for_service(srv, timeout=self.timeout)
            results[i] = rospy.ServiceProxy(srv, Trigger)()
            (rospy.loginfo if results[i].success else rospy.logwarn)(
                "[recorder_mux] %s %s: %s", srv, action, results[i].message)
        except Exception as e:  # unavailable camera must never fail the whole trigger
            rospy.logwarn("[recorder_mux] %s %s failed: %s", srv, action, e)

    def _all_ok(self, results):
        return bool(results) and all(r is not None and r.success for r in results)


def _master_watchdog():
    """Re-register onto a restarted Jetson roscore (see recorder_node): rospy stays
    bound to the dead master and never re-advertises /recorder/start, so the onboard
    trigger silently finds nothing. Exit on a master-PID change; systemd relaunches."""
    m = rospy.get_master()
    base = None
    while not rospy.is_shutdown():
        try:
            pid = m.getPid()[2]
            if base is None:
                base = pid
            elif pid != base:
                rospy.logwarn("[recorder_mux] ROS master restarted (pid %s->%s) — exiting to re-register", base, pid)
                rospy.signal_shutdown("ros master restarted")
                time.sleep(2)
                os._exit(1)
        except Exception:
            pass
        time.sleep(5)


def main():
    rospy.init_node("recorder")   # -> services /recorder/start, /recorder/stop
    RecorderMux()
    threading.Thread(target=_master_watchdog, daemon=True).start()
    rospy.spin()


if __name__ == "__main__":
    main()
