"""One timed motor-duty boost for each confirmed hill sighting."""


class HillBoost:
    def __init__(self, confirm_sec=2.0, boost_sec=2.0, rearm_sec=0.5):
        self.confirm_sec = float(confirm_sec)
        self.boost_sec = float(boost_sec)
        self.rearm_sec = float(rearm_sec)
        self.visible_since = None
        self.absent_since = None
        self.boost_until = 0.0
        self.used = False

    def update(self, visible, now):
        now = float(now)
        if visible:
            self.absent_since = None
            if self.visible_since is None:
                self.visible_since = now
            if not self.used and now - self.visible_since >= self.confirm_sec:
                self.boost_until = now + self.boost_sec
                self.used = True
        else:
            self.visible_since = None
            if self.absent_since is None:
                self.absent_since = now
            if now - self.absent_since >= self.rearm_sec:
                self.used = False
        return now < self.boost_until
