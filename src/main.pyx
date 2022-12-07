# from __future__ import annotations
STUFF = "Hi"
from libc.math cimport sqrt, atan2, copysign, cos, sin, M_PI, round, pow

import numpy as np
cimport numpy as np

cdef class Collision:
    cdef Unit a
    cdef Unit b
    cdef float t

    def __init__(self, Unit a, Unit b, float t):
        self.a = a
        self.b = b
        self.t = t

cdef class Point:
    cdef public float x
    cdef public float y

    def __init__(self, float x, float y):
        self.x = x
        self.y = y

    cpdef float dist(self, Point p):
        return np.sqrt(pow((self.x - p.x), 2) + pow((self.y - p.y), 2))

    cdef float dist2(self, Point p):
        return pow((self.x - p.x), 2) + pow((self.y - p.y), 2)

    cdef Point closest(self, Point a, Point b):
        cdef float da = b.y - a.y
        cdef float db = a.x - b.x
        cdef float c1 = da * a.x + db * a.y
        cdef float c2 = -db * self.x + da * self.y
        cdef float det = da * da + db * db
        cdef float cx
        cdef float cy

        if det != 0:
            cx = (da * c1 - db * c2) / det
            cy = (da * c2 + db * c1) / det
        else:
            cx = self.x
            cy = self.y

        return Point(cx, cy)

cdef class Unit(Point):

    cdef np.ndarray cache
    cdef public float vx
    cdef public float vy
    cdef public int id
    cdef public int type
    cdef public int r

    def __init__(self, float x, float y):
        Point.__init__(self, x, y)
        self.cache = np.zeros(4)
        self.vx = 0
        self.vy = 0

    cdef void bounce(self, Unit u):
        pass

    cdef void save(self):
        self.cache[0] = self.x
        self.cache[1] = self.y
        self.cache[2] = self.vx
        self.cache[3] = self.vy

    cdef void load(self):
        self.x = self.cache[0]
        self.y = self.cache[1]
        self.vx = self.cache[2]
        self.vy = self.cache[3]

cdef class Checkpoint(Unit):

    def __init__(self, id_, x, y):
        Unit.__init__(self, x, y)
        self.id = id_

        self.vx = self.vy = 0
        self.type = 0
        self.r = 600

    cdef void bounce(self, Unit u):
        pass

cdef class Pod(Unit):

    cdef public int ncpid
    cdef public short timeout
    cdef bint has_boost
    cdef public short checked
    cdef public short shield
    cdef public float angle
    cdef float next_angle
    cdef short max_angle
    cdef Pod partner
    cdef public short laps
    cdef short cp_ct
    cdef list cps

    def __init__(self, id_, cps, laps):
        Unit.__init__(self, 0, 0)
        self.id = id_
        self.r = 400
        self.type = 1
        self.ncpid = 1

        self.timeout = 100
        self.has_boost = True
        self.checked = 0
        self.shield = 0
        self.angle = -1
        self.next_angle = -1
        self.cache = np.zeros(6)
        self.partner = None
        self.laps = laps
        self.cp_ct = len(cps)
        self.cps = cps

    cpdef score(self):
        cdef Checkpoint cp = self.cps[self.ncpid]
        return self.dist(cp)

    cpdef float collision_time(self, Unit u):
        cdef Point p
        cdef float dist
        cdef float pdist
        cdef float mypdist
        cdef float sr
        cdef float length
        cdef float backdist
        cdef float t
        cdef float x
        cdef float y
        cdef float vx
        cdef float vy
        cdef Point myp
        cdef Point up

        # Square of the distance
        dist = self.dist2(u)

        # Sum of the radii squared
        sr = (self.r + u.r) * (self.r + u.r)

        # We take everything squared to avoid calling sqrt uselessly. It is better for performances

        if dist < sr:
            # Objects are already touching each other. We have an immediate collision.
            return 0.0

        # We place ourselves in the reference frame of u. u is therefore stationary and is at (0,0)
        x = self.x - u.x
        y = self.y - u.y
        myp = Point(x, y)
        vx = self.vx - u.vx
        vy = self.vy - u.vy
        up = Point(0, 0)

        # We look for the closest to u (which is in (0,0)) on the line described by our speed vector
        p = up.closest(myp, Point(x + vx, y + vy))

        # Square of the distance between u and the closest to u on the line described by our speed vector
        pdist = up.dist2(p)

        # Square of the distance between us and that point
        mypdist = myp.dist2(p)

        # If the distance between u and self line is less than the sum of the radii, there might be a collision
        if pdist < sr:
            # Our speed on the line
            length = np.sqrt(vx * vx + vy * vy)

            if length < 1e-6:
                return -1

            # We move along the line to find the of impact
            backdist = np.sqrt(sr - pdist)
            p.x = p.x - backdist * (vx / length)
            p.y = p.y - backdist * (vy / length)

            # If the is now further away it means we are not going the right way, therefore the collision won't happen
            if myp.dist2(p) > mypdist:
                return -1

            pdist = p.dist(myp)

            # The of impact is further than what we can travel in one turn
            if pdist > length:
                return -1

            # Time needed to reach the impact point
            t = pdist / length

            return t

        return -1

    cpdef void apply(self, float thrust, float angle):
        self.angle += max(-18., min(18., angle))
        if self.angle >= 360.:
            self.angle = self.angle - 360.
        elif self.angle < 0.0:
            self.angle += 360.

        if thrust == -1:
            self.shield = 4
        else:
            self.boost(thrust)

    cdef void rotate(self, Point p):
        self.angle += max(-18., min(18., self.diff_angle(p)))
        if self.angle >= 360.:
            self.angle = self.angle - 360.
        elif self.angle < 0.0:
            self.angle += 360.

    cdef void boost(self, float thrust):
        cdef float ra
        if self.shield > 0:
            return

        ra = self.angle * np.pi / 180.0

        self.vx += np.cos(ra) * thrust
        self.vy += np.sin(ra) * thrust

    cpdef void move(self, float t):
        self.x += self.vx * t
        self.y += self.vy * t

    cpdef void end(self):
        self.x = round(self.x)
        self.y = round(self.y)
        self.vx = int(self.vx * 0.85)
        self.vy = int(self.vy * 0.85)
        if self.checked >= self.cp_ct * self.laps:
            self.ncpid = 0
            self.checked = self.cp_ct * self.laps

        self.timeout -= 1
        if self.shield > 0:
            self.shield -= 1

    cdef void bounce(self, Unit u):
        if u.type == 0:  # check if type checkpoint
            self.checked += 1
            self.timeout = 100
            if self.partner != None:
                self.partner.timeout = 100
            self.ncpid = (self.ncpid + 1) % self.cp_ct
            return

        self.bounce_w_pod(u)

    cdef bounce_w_pod(self, Unit u):
        m1 = 10 if self.shield == 4 else 1.
        m2 = 10 if u.shield == 4 else 1.
        mcoeff = (m1 + m2) / (m1 * m2)

        nx = self.x - u.x
        ny = self.y - u.y
        dst2 = nx * nx + ny * ny
        dvx = self.vx - u.vx
        dvy = self.vy - u.vy
        prod = (nx * dvx + ny * dvy) / (dst2 * mcoeff)
        fx = nx * prod
        fy = ny * prod
        m1_inv = 1.0 / m1
        m2_inv = 1.0 / m2

        self.vx -= fx * m1_inv
        self.vy -= fy * m1_inv
        u.vx += fx * m2_inv
        u.vy += fy * m2_inv

        impulse = np.sqrt(fx * fx + fy * fy)
        if impulse < 120.:
            df = 120.0 / impulse
            fx *= df
            fy *= df

        self.vx -= fx * m1_inv
        self.vy -= fy * m1_inv
        u.vx += fx * m2_inv
        u.vy += fy * m2_inv

    cpdef diff_angle(self, p):
        a = self.get_angle(p)
        right = a - self.angle if self.angle <= a else 360. - self.angle + a
        left = self.angle - a if self.angle >= a else self.angle + 360. - a

        if right < left:
            return right

        return -left

    cdef get_angle(self, p):
        d = self.dist(p)
        if d == 0:
            d = 0.00001
        dx = (p.x - self.x) / d
        dy = (p.y - self.y) / d

        a = np.arccos(dx) * 180 / np.pi

        if dy < 0:
            a = 360 - a

        return a

    cdef update(self, x, y, vx, vy, angle, ncpid):
        if self.shield > 0:
            self.shield -= 1
        if ncpid != self.ncpid:
            self.timeout = self.partner.timeout = 100
            self.checked += 1
        else:
            self.timeout -= 1

        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.ncpid = ncpid

        is_p2 = False
        if is_p2 and self.id > 1:
            self.swap(angle, self.next_angle)
        self.angle = angle
        r = -1
        if r == 0:
            self.angle = 1 + self.diff_angle(self.cps[1])
        self.save()

    cdef void save(self):
        super().save()
        self.cache[0] = self.ncpid
        self.cache[1] = self.checked
        self.cache[2] = self.timeout
        self.cache[3] = self.shield
        self.cache[4] = self.angle
        self.cache[5] = self.has_boost

    cdef void load(self):
        self.load()
        self.ncpid = self.cache[0]
        self.checked = self.cache[1]
        self.timeout = self.cache[2]
        self.shield = self.cache[3]
        self.angle = self.cache[4]
        self.has_boost = self.cache[5]

    def __str__(self):
        print("Velocity: (" + str(self.vx) + "," + str(self.vy) + ")")

level_pool = []
ch0 = []
ch0.append(Checkpoint(0, 13939, 1936))
ch0.append(Checkpoint(1, 7996, 3277))
ch0.append(Checkpoint(2, 2642, 7014))
ch0.append(Checkpoint(3, 10025, 5946))
level_pool.append(ch0)

ch1 = []
ch1.append(Checkpoint(0, 7828, 836))
ch1.append(Checkpoint(1, 7659, 5995))
ch1.append(Checkpoint(2, 3169, 7536))
ch1.append(Checkpoint(3, 9495, 4351))
ch1.append(Checkpoint(4, 14516, 7793))
ch1.append(Checkpoint(5, 6331, 4292))
level_pool.append(ch1)

ch2 = []
ch2.append(Checkpoint(0, 13868, 1212))
ch2.append(Checkpoint(1, 10243, 4906))
ch2.append(Checkpoint(2, 6075, 2209))
ch2.append(Checkpoint(3, 3035, 5181))
ch2.append(Checkpoint(4, 6274, 7779))
ch2.append(Checkpoint(5, 14112, 7758))
level_pool.append(ch2)

ch3 = []
ch3.append(Checkpoint(0, 14679, 1407))
ch3.append(Checkpoint(1, 3431, 7235))
ch3.append(Checkpoint(2, 9424, 7225))
ch3.append(Checkpoint(3, 5990, 4252))
level_pool.append(ch3)

ch4 = []
ch4.append(Checkpoint(0, 13506, 2367))
ch4.append(Checkpoint(1, 12945, 7247))
ch4.append(Checkpoint(2, 5646, 2586))
ch4.append(Checkpoint(3, 4118, 7442))
level_pool.append(ch4)

ch5 = []
ch5.append(Checkpoint(0, 11495, 6084))
ch5.append(Checkpoint(1, 9127, 1863))
ch5.append(Checkpoint(2, 4984, 5248))
level_pool.append(ch5)

ch6 = []
ch6.append(Checkpoint(0, 13575, 7620))
ch6.append(Checkpoint(1, 12478, 1332))
ch6.append(Checkpoint(2, 10562, 6005))
ch6.append(Checkpoint(3, 3577, 5201))
level_pool.append(ch6)

cdef class Sim:

    cdef public short laps
    cdef public list pods
    cdef public list checkpoints

    def __init__(self, list checkpoints):
        self.checkpoints = checkpoints
        # cps = level_pool[np.random.randint(len(level_pool))]
        self.laps = 2
        self.pods = self.init_pods(self.checkpoints, self.laps)

    cdef init_pods(self, checkpoints, laps):
        pods = []

        start_points = []
        start_points.append(Point(0, 1000))
        start_points.append(Point(1000, 100))
        np.random.shuffle(start_points)

        for i in range(2):
            p = Pod(i, checkpoints, laps)
            p.x = start_points[i].x
            p.y = start_points[i].y
            p.vx = 0
            p.vy = 0
            p.angle = 0
            pods.append(p)
        return pods

    cpdef bint is_done(self):
        for pod in self.pods:
            if pod.checked == int(len(self.checkpoints) * self.laps) or pod.timeout < 0:
                return True
        return False

    def on_round(self):
        t = 0.0
        last_col = None
        while t < 1.0:
            first_col = Collision(None, None, -1)
            for i in range(len(self.pods)):
                for j in range(i + 1, len(self.pods)):
                    col_time = self.pods[i].collision_time(self.pods[j])
                    if col_time > -1 and col_time + t < 1.0 and (first_col.t == -1 or col_time < first_col.t):
                        if last_col is None or not (col_time == 0 and last_col.a == self.pods[i] and last_col.b == self.pods[j]):
                            first_col.a = self.pods[i]
                            first_col.b = self.pods[j]
                            first_col.t = col_time
                            last_col = first_col

                # TODO this is wasteful, get rid of it
                col_time = self.pods[i].collision_time(self.checkpoints[self.pods[i].ncpid])
                if col_time > -1 and col_time + t < 1.0 and (first_col.t == -1 or col_time < first_col.t):
                    first_col.a = self.pods[i]
                    first_col.b = self.checkpoints[self.pods[i].ncpid]
                    first_col.t = col_time

            if first_col.t == -1:
                for i in range(len(self.pods)):
                    self.pods[i].move(1.0 - t)

                t = 1.0
            else:
                for i in range(len(self.pods)):
                    self.pods[i].move(first_col.t)

                first_col.a.bounce(first_col.b)
                t += first_col.t

        for i in range(len(self.pods)):
            self.pods[i].end()
