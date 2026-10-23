"""Regression replay and safety checks; never injects game input."""
import glob
import json
import unittest
from throw_sequence import ThrowRecognizer


class SequenceTests(unittest.TestCase):
    def test_loss_clears_preparation(self):
        rec = ThrowRecognizer()
        rec.arms['right'] = {'mode': 'forward'}
        self.assertIsNone(rec.update([], 1.))
        self.assertFalse(rec.arms)

    def test_short_pose_is_rejected(self):
        self.assertIsNone(ThrowRecognizer().update([[0., 0., 0., 1.]], 1.))

    def test_pivot_hold_and_fresh_preparation(self):
        for dt in (.04, .2):
            rec = ThrowRecognizer()
            points = [[.5, .5, 0., 1.] for _ in range(33)]
            points[11] = [.4, .3, 0., 1.]
            points[12] = [.6, .3, 0., 1.]
            points[14] = [.6, .38, 0., 1.]
            events = []
            for i, wrist_y in enumerate([.4]*5 + [.52]*10 + [.4]*5 + [.52]*5):
                points[16] = [.6, wrist_y, 0., 1.]
                event = rec.update(points, i*dt)
                if event:
                    events.append(event)
            self.assertEqual(events, ['backward', 'backward'])

    def test_labelled_replay(self):
        files = glob.glob('throw-training/*.jsonl')
        if not files:
            self.skipTest('recordings remain on tracker host')
        for path in sorted(files):
            if '195431-' in path:
                continue
            rec = ThrowRecognizer()
            events = []
            with open(path) as recording:
                for line in recording:
                    row = json.loads(line)
                    if row.get('type') == 'sample':
                        event = rec.update(row['landmarks'], row['t'])
                        if event:
                            events.append(event)
            print(path, {e: events.count(e) for e in set(events)})
            if 'normal_movement' in path:
                self.assertEqual(events, [], path)
            elif 'overarm' in path:
                self.assertGreaterEqual(events.count('forward'), 7, path)
                self.assertNotIn('backward', events, path)
            elif 'underarm' in path:
                self.assertGreaterEqual(events.count('backward'), 5, path)
                self.assertNotIn('forward', events, path)
                if '202424-' in path:
                    self.assertEqual(events.count('backward'), 10, path)


if __name__ == '__main__':
    unittest.main()
