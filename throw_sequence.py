"""Image-plane throw sequences; depth is deliberately not a trigger feature."""
from collections import deque
import math
import statistics


class ThrowRecognizer:
    def __init__(self):
        self.arms = {}
        self.last_fire = -10.0

    def reset(self):
        self.arms.clear()

    def update(self, points, now, stroke=.25, minimum_visibility=.4):
        if not points or len(points) < 33:
            self.reset()
            return None
        width = math.hypot(points[11][0]-points[12][0], points[11][1]-points[12][1])
        if width < .03:
            self.reset()
            return None
        event = None
        for side, s, e, w in [('left', 11, 13, 15), ('right', 12, 14, 16)]:
            if min(points[i][3] for i in (s, e, w)) < minimum_visibility:
                self.arms.pop(side, None)
                continue
            raw = (points[w][1]-points[s][1])/width
            a = self.arms.get(side)
            if a is None or not 0 < now-a['time'] <= .25:
                self.arms[side] = dict(time=now, history=deque([raw], maxlen=3),
                    y=raw, mode='idle', low_since=None, ready=False,
                    origin=raw, peak_speed=0., ascent_start=now,
                    elbow_history=deque(maxlen=3), pivot_armed=False,
                    pivot_allowed=True, pivot_origin=0.)
                continue
            a['history'].append(raw)
            y = statistics.median(a['history'])
            velocity = (y-a['y'])/(now-a['time'])
            a['time'], a['y'] = now, y
            a['elbow_history'].append((points[e][1]-points[s][1])/width)
            elbow_y = statistics.median(a['elbow_history'])
            forearm_y = y-elbow_y
            # A neutral lowered arm ends the previous overarm sequence. Its
            # follow-through must not become a second, backwards item request.
            if y > 1.2 and elbow_y > .7:
                a['pivot_allowed'] = True
            if a['mode'] == 'spent':
                a['return_peak'] = max(a.get('return_peak', y), y)
            # A fresh low preparation rearms, not an arbitrary cooldown.
            if y > 1.2:
                if a['low_since'] is None:
                    a['low_since'] = now
                if now-a['low_since'] >= .25:
                    a['ready'] = True
                    if a['mode'] == 'spent':
                        a['mode'] = 'idle'
            else:
                a['low_since'] = None
            if y < -.25:
                a['pivot_armed'] = False
                a['pivot_allowed'] = False
                # An overhead wind-up always cancels a possible underarm lift.
                if a['mode'] != 'forward' and a['mode'] != 'spent':
                    a['mode'], a['origin'] = 'forward', y
                elif a['mode'] == 'forward':
                    a['origin'] = min(a['origin'], y)
                elif velocity < -.5 and a.get('return_peak', y)-y >= .7:
                    a['mode'], a['origin'] = 'forward', y
                a['ready'] = False
            firing = None
            if a['mode'] == 'forward':
                if y-a['origin'] >= max(.45, stroke) and velocity > 3.:
                    firing = 'forward'
                elif y > 1.2:
                    a['mode'] = 'idle'
            elif a['pivot_allowed']:
                # Preparation: elbow elevated, wrist roughly at elbow height.
                # Throw: the forearm pivots below the still-elevated elbow.
                # No speed threshold or return stroke is required, so slower
                # deliberate throws remain valid. Fresh preparation rearms.
                if elbow_y < .65 and -.4 < forearm_y < .3 and y > -.1:
                    if not a['pivot_armed']:
                        a['pivot_origin'] = forearm_y
                    a['pivot_armed'] = True
                elif (a['pivot_armed'] and elbow_y < .65 and forearm_y > .6
                      and forearm_y-a['pivot_origin'] >= max(.1, stroke)*1.8):
                    firing = 'backward'
                    a['pivot_armed'] = False
                elif elbow_y > .8:
                    a['pivot_armed'] = False
            if firing:
                a['mode'] = 'spent'
                a['ready'] = False
                a['low_since'] = None
                a['return_peak'] = y
                if event is None and now-self.last_fire >= .12:
                    event, self.last_fire = firing, now
        return event
