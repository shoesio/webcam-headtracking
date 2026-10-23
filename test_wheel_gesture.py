import unittest
from wheel_gesture import WheelGesture, gamecube_steering, steering_ramp
from wheel_profile import profile
import struct


class WheelTests(unittest.TestCase):
    def test_interpolation_reaches_full_lock_without_overshoot(self):
        w=WheelGesture(interpolation_ms=30)
        cfg={'steering_enabled':1,'steering_smoothing_ms':0,'steering_gain':25}
        w.update(self.pose(),1.,cfg,received_time=1.);w.tick(1.)
        w.update(self.pose(.02),1.05,cfg,received_time=1.06)
        self.assertEqual(w.tick(1.06),0)
        self.assertAlmostEqual(w.tick(1.075),-.5)
        self.assertAlmostEqual(w.tick(1.091),-1)
        self.assertEqual(w.tick(1.1),-1)
        w.update([],1.11,cfg,received_time=1.11)
        self.assertEqual(w.tick(2.),-1)

    def test_loss_cancels_transition_at_current_output(self):
        w=WheelGesture(interpolation_ms=30)
        cfg={'steering_enabled':1,'steering_smoothing_ms':0,'steering_gain':25}
        w.update(self.pose(),1.,cfg);w.tick(1.)
        w.update(self.pose(.02),1.05,cfg);w.tick(1.06)
        held=w.value
        w.update([],1.065,cfg);w.tick(1.08)
        self.assertAlmostEqual(w.value,held)

    def test_ramps_keep_center_motion_and_full_lock(self):
        for mode in (0,1,2):
            self.assertEqual(steering_ramp(0,mode),0)
            self.assertEqual(steering_ramp(1,mode),1)
            self.assertEqual(steering_ramp(-1,mode),-1)
            self.assertGreater(steering_ramp(.0001,mode),0)
            values=[steering_ramp(i/100,mode) for i in range(-100,101)]
            self.assertTrue(all(b>a for a,b in zip(values,values[1:])))
        self.assertAlmostEqual(steering_ramp(.5,1),.275)
        self.assertLess(steering_ramp(.5,2),steering_ramp(.5,1))

    def test_curve_hot_reload_preserves_direction_and_neutral(self):
        w=WheelGesture()
        cfg={'steering_enabled':1,'steering_smoothing_ms':0,'steering_gain':25,'steering_curve':0}
        w.update(self.pose(.01),1.,cfg);w.tick(1.)
        self.assertAlmostEqual(w.value,-.5)
        cfg['steering_curve']=1
        w.update(self.pose(.01),1.1,cfg);w.tick(1.1)
        self.assertAlmostEqual(w.value,-.275)
        cfg['steering_curve']=2
        w.update(self.pose(.01),1.2,cfg);w.tick(1.2)
        self.assertLess(abs(w.value),.275)
        w.update(self.pose(),1.3,cfg);w.tick(1.3)
        self.assertEqual(w.value,0)
    def test_manual_strength_ignores_angle_setting(self):
        for angle in (5,20,90):
            w=WheelGesture()
            cfg={'steering_enabled':1,'steering_smoothing_ms':0,'steering_full_turn_deg':angle}
            w.update(self.pose(),1.,cfg);w.tick(1.)
            for i,offset in enumerate((.0001,.04,-.0001,-.04,0)):
                w.update(self.pose(offset),1.1+i*.1,cfg);w.tick(1.1+i*.1)
                self.assertAlmostEqual(w.value,max(-1,min(1,-offset*2*20)))

    def test_manual_strength_is_proportional(self):
        w=WheelGesture();cfg={'steering_enabled':1,'steering_smoothing_ms':0,'steering_gain':10}
        w.update(self.pose(.01),1.,cfg);w.tick(1.)
        self.assertAlmostEqual(w.value,-.2)
        w.update(self.pose(.02),1.1,cfg);w.tick(1.1)
        self.assertAlmostEqual(w.value,-.4)
        cfg['steering_gain']=20
        w.update(self.pose(.02),1.2,cfg);w.tick(1.2)
        self.assertAlmostEqual(w.value,-.8)

    def test_no_deadzone_profile(self):
        p=profile({})
        self.assertEqual(len(p),302)
        self.assertEqual(struct.unpack_from('<II',p),(0x4c525443,3))
        self.assertFalse(p[33])
        self.assertEqual(struct.unpack_from('<HH',p,34),(0,0))

    def test_standard_full_range_and_neutral(self):
        self.assertEqual(gamecube_steering(0),0)
        for value in (.0001,.01,.1,.5,1):
            self.assertGreater(gamecube_steering(value),0)
            self.assertLess(gamecube_steering(-value),0)
        self.assertEqual(gamecube_steering(1),32767)
        self.assertEqual(gamecube_steering(-1),-32768)
        self.assertEqual(gamecube_steering(1e-12),0)
        self.assertEqual(gamecube_steering(-1e-12),0)
        self.assertEqual(gamecube_steering(.5),16384)
    def pose(self, tilt=0):
        p = [[.5,.5,0.,1.] for _ in range(33)]
        p[11]=[.65,.3,0.,1.];p[12]=[.35,.3,0.,1.]
        p[13]=[.65,.5,0.,1.];p[14]=[.35,.5,0.,1.]
        p[15]=[.65,.5+tilt,0.,1.];p[16]=[.35,.5-tilt,0.,1.]
        return p

    def test_steer_recenter_and_disable(self):
        w=WheelGesture();cfg={'steering_enabled':1,'steering_smoothing_ms':0}
        w.update(self.pose(.04),1.,cfg);w.tick(1.)
        self.assertLess(w.value,0)
        cfg['steering_center_request']=1
        w.update(self.pose(.04),1.1,cfg);w.tick(1.1)
        self.assertLess(w.value,0)
        w.update(self.pose(),1.15,cfg);w.tick(1.15)
        self.assertAlmostEqual(w.value,0)
        cfg['steering_enabled']=0
        w.update(self.pose(.04),1.2,cfg);w.tick(1.2)
        self.assertEqual(w.value,0)

    def test_tracking_loss_holds_until_recovery_or_disable(self):
        w=WheelGesture();cfg={'steering_enabled':1,'steering_smoothing_ms':0}
        w.update(self.pose(.04),1.,cfg);w.tick(1.)
        held=w.value
        p=[]
        w.update(p,1.1,cfg);w.tick(1.2)
        self.assertEqual(w.value,held)
        w.tick(2.)
        self.assertEqual(w.value,held)
        w.tick(30.)
        self.assertEqual(w.value,held)
        w.update(self.pose(-.04),30.1,cfg);w.tick(30.1)
        self.assertGreater(w.value,0)
        w.enabled=False;w.tick(30.2)
        self.assertEqual(w.value,0)

    def test_smoothing_does_not_continue_during_loss(self):
        w=WheelGesture();cfg={'steering_enabled':1,'steering_smoothing_ms':100}
        w.update(self.pose(.04),1.,cfg);w.tick(1.)
        w.tick(1.05)
        held=w.value
        w.update([],1.06,cfg);w.tick(1.1);w.tick(2.)
        self.assertEqual(w.value,held)

    def test_tilt_is_not_tracking_loss(self):
        w=WheelGesture();cfg={'steering_enabled':1,'steering_smoothing_ms':0}
        w.update(self.pose(),1.,cfg);w.tick(1.)
        p=self.pose()
        p[15][:2]=[.5,.7];p[16][:2]=[.5,.3]
        w.update(p,1.1,cfg);w.tick(1.1)
        self.assertEqual(w.reason,'tracking wheel')
        self.assertNotEqual(w.value,0)

    def test_height_mapping_independent_of_live_horizontal_spacing(self):
        w=WheelGesture()
        cfg={'steering_enabled':1,'steering_smoothing_ms':0,'steering_center_request':1}
        w.update(self.pose(),1.,cfg);w.tick(1.)
        p=self.pose(.04)
        w.update(p,1.1,cfg);w.tick(1.1)
        original=w.value
        p[15][0]=.6;p[16][0]=.4
        w.update(p,1.2,cfg);w.tick(1.2)
        self.assertAlmostEqual(w.value,original)
        p[15][1]+=.05;p[16][1]+=.05
        w.update(p,1.3,cfg);w.tick(1.3)
        self.assertAlmostEqual(w.value,original)

    def test_small_height_change_has_no_deadzone(self):
        w=WheelGesture()
        cfg={'steering_enabled':1,'steering_smoothing_ms':0,'steering_center_request':1}
        w.update(self.pose(),1.,cfg);w.tick(1.)
        w.update(self.pose(.0001),1.1,cfg);w.tick(1.1)
        self.assertLess(w.value,0)

    def test_centre_button_cannot_move_picture_neutral(self):
        w=WheelGesture()
        cfg={'steering_enabled':1,'steering_smoothing_ms':0,'steering_center_request':123}
        w.update(self.pose(.04),1.,cfg);w.tick(1.)
        self.assertLess(w.value,0)
        w.update(self.pose(),1.1,cfg);w.tick(1.1)
        self.assertEqual(w.value,0)

    def test_small_controller_and_uncertain_elbows_still_steer(self):
        w=WheelGesture()
        cfg={'steering_enabled':1,'steering_smoothing_ms':0}
        w.update(self.pose(),1.,cfg);w.tick(1.)
        for i,offset in enumerate((.02,-.02)):
            p=self.pose()
            p[15][:2]=[.53,.5+offset];p[16][:2]=[.47,.5-offset]
            p[13][3]=p[14][3]=.1
            w.update(p,1.1+i*.1,cfg);w.tick(1.1+i*.1)
            self.assertEqual(w.reason,'tracking wheel')
            self.assertGreater(abs(w.value),0)
            self.assertEqual(w.value<0,offset>0)

    def test_occluded_wrist_estimates_are_not_blocked(self):
        w=WheelGesture();cfg={'steering_enabled':1,'steering_smoothing_ms':0}
        for i,offset in enumerate((.04,-.04)):
            p=self.pose(offset);p[15][3]=p[16][3]=0.
            w.update(p,1.+i*.1,cfg);w.tick(1.+i*.1)
            self.assertEqual(w.reason,'tracking wheel')
            self.assertEqual(w.value,-1 if offset>0 else 1)


if __name__=='__main__':
    unittest.main()
