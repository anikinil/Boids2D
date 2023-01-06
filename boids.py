import pyglet as pg
from random import uniform
from math import acos, sin, cos, pi, sqrt

# window + clear

win_w = 1800
win_h = 900
mid_x = win_w / 2
mid_y = win_h / 2
margin = 100

update_dt = 0.02

win = pg.window.Window(win_w, win_h)

clear_r = 30
clear_g = 30
clear_b = 30
clear_a = 255

pg.gl.glClearColor(clear_r/255, clear_g/255, clear_b/255, clear_a/200)

# mouse

mouse_x = 0
mouse_y = 0

# boids

boid_num = 150
boid_size = 5
boid_speed = 10
boid_steering_speed = 0.01
boid_clr = (100, 180, 250)

alignment_r = 250
alignment_fact = 4
cohesion_r = 180
cohesion_fact = 1
separation_r = 80
separation_fact = 2.5

boid_vision_r = max(alignment_r, cohesion_r, separation_r)

mouse_avoidance__r = 80
mouse_avoidance_fact = 4

mouse_outline = pg.shapes.Circle(mouse_x, mouse_y, mouse_avoidance__r)
mouse_outline.color = (255, 190, 0, 50)

edge_avoidance_r = 80
edge_avoidance_fact = 6

noise_fact = 3

# utils

def get_angle(vec_x, vec_y):

    vec_z_x = 0
    vec_z_y = 1

    var = (vec_z_x*vec_x + vec_z_y*vec_y)/(magn(vec_z_x, vec_z_y)*magn(vec_x, vec_y))

    if var <= -1:
        angle = pi
    elif var >= 1:
        angle = 0
    else:
        angle = acos(var)

    return angle if vec_x >= 0 else -angle

def magn(vec_x, vec_y):

    return sqrt(vec_x ** 2 + vec_y ** 2)

def dist(x1, y1, x2, y2):

    return magn(x2 - x1, y2 - y1)

def norm(vec_x, vec_y):

    magn_var = magn(vec_x, vec_y)

    return (vec_x/magn_var, vec_y/magn_var)

def rot(vec_x, vec_y, theta):

    res_x = vec_x * cos(theta) - vec_y * sin(theta)
    res_y = vec_x * sin(theta) + vec_y * cos(theta)

    return (res_x, res_y)

def get_rand_pos(margin):

    rand_x = uniform(margin, win_w-margin)
    rand_y = uniform(margin, win_h-margin)

    return (rand_x, rand_y)

def get_rand_dir(boid_speed):

    dir_x = uniform(-1, 1)
    dir_y = uniform(-1, 1)

    norm_var = norm(dir_x, dir_y)

    x = norm_var[0] * boid_speed
    y = norm_var[1] * boid_speed

    return (x, y)

def visible(b1, b2):

    if dist(b1.pos_x, b1.pos_y, b2.pos_x, b2.pos_y) <= boid_vision_r:
        return True
        
    return False

def close_enough(b1, b2, dist):

    if magn(b2.pos_x - b1.pos_x, b2.pos_y - b1.pos_y) <= dist:
        
        return True
    
    return False

def get_midpoint(boids):

    n = len(boids)
    sum_x = 0
    sum_y = 0

    for b in boids:
        sum_x += b.pos_x
        sum_y += b.pos_y

    return (sum_x/n, sum_y/n)

def get_aver_dir(boids):

    n = len(boids)
    sum_x = 0
    sum_y = 0

    for b in boids:
        sum_x += b.dir_x
        sum_y += b.dir_y

    return (sum_x/n, sum_y/n)

def separate(b1, separation_boids):

    if len(separation_boids) > 0:

        midpoint = get_midpoint(separation_boids)
        b1.steer_to(b1.pos_x - midpoint[0], b1.pos_y - midpoint[1], separation_fact)

def align(b1, alignment_boids):

    if len(alignment_boids) > 0:

        aver_dir = get_aver_dir(alignment_boids)
        b1.steer_to(aver_dir[0], aver_dir[1], alignment_fact)

def cohese(b1, cohesion_boids):
    
    if len(cohesion_boids) > 0:

        midpoint = get_midpoint(cohesion_boids)
        b1.steer_to(midpoint[0] - b1.pos_x, midpoint[1] - b1.pos_y, cohesion_fact)

def avoid_mouse(b1):

    if dist(b1.pos_x, b1.pos_y, mouse_x, mouse_y) <= mouse_avoidance__r:
        b1.steer_to(b1.pos_x - mouse_x, b1.pos_y - mouse_y, mouse_avoidance_fact)
    
def avoid_edges(b1):

    steer_x = 0
    steer_y = 0

    if b1.pos_x <= edge_avoidance_r:
        steer_x += 50
        
    if b1.pos_x >= win_w - edge_avoidance_r:
        steer_x -= 50

    if b1.pos_y <= edge_avoidance_r:
        steer_y += 50

    if b1.pos_y >= win_h - edge_avoidance_r:
        steer_y -= 50

    if steer_x != 0 or steer_y != 0:
        b1.steer_to(steer_x, steer_y, edge_avoidance_fact)

def add_noise(b1):

    pos = get_rand_dir(boid_speed)
    b1.steer_to(pos[0], pos[1], noise_fact)

class Boid:

    def __init__(self, pos_x, pos_y, dir_x, dir_y, speed, clr):

        self.pos_x = pos_x
        self.pos_y = pos_y
        self.dir_x = dir_x
        self.dir_y = dir_y
        self.speed = speed
        self.clr = clr

    def steer_to(self, des_dir_x, des_dir_y, fact):

        steer_dir_x = (des_dir_x - self.dir_x) * boid_steering_speed * fact
        steer_dir_y = (des_dir_y - self.dir_y) * boid_steering_speed * fact
        
        norm_var = norm(self.dir_x + steer_dir_x, self.dir_y + steer_dir_y)
        
        new_dir_x = norm_var[0] * self.speed
        new_dir_y = norm_var[1] * self.speed

        self.dir_x = new_dir_x
        self.dir_y = new_dir_y

    @property
    def get_shape(self):

        vert1_x = self.pos_x
        vert1_y = self.pos_y + 3 * boid_size
        vert2_x = self.pos_x - 1 * boid_size
        vert2_y = self.pos_y - 1 * boid_size
        vert3_x = self.pos_x
        vert3_y = self.pos_y
        vert4_x = self.pos_x + 1 * boid_size
        vert4_y = self.pos_y - 1 * boid_size

        shp = pg.shapes.Polygon([vert1_x, vert1_y], 
                                [vert2_x, vert2_y], 
                                [vert3_x, vert3_y],
                                [vert4_x, vert4_y])

        shp.rotation = get_angle(self.dir_x, self.dir_y) / (2*pi) * 360
        shp.color = self.clr

        return shp

boids = []

boids.append(
    Boid(
        mid_x, mid_y,
        0, -1,
        boid_speed,
        boid_clr
    )
)

for i in range(boid_num):

    pos = get_rand_pos(margin)
    dir = get_rand_dir(boid_speed)

    boid = Boid(
        pos[0],
        pos[1],
        dir[0],
        dir[1],
        boid_speed,
        boid_clr)

    boids.append(boid)

@win.event
def on_draw():

    win.clear()

    for b in boids:
        b.get_shape.draw()

    mouse_outline.draw()

@win.event
def on_mouse_motion(x, y, dx, dy):
    global mouse_x, mouse_y

    mouse_x = x
    mouse_y = y
        
def update(dt):

    for b1 in boids:

        if b1.pos_x > win_w:
            b1.pos_x = 0        
        if b1.pos_x < 0:
            b1.pos_x = win_w
            
        if b1.pos_y > win_h:
            b1.pos_y = 0        
        if b1.pos_y < 0:
            b1.pos_y = win_h

        visible_boids = []
        for b2 in boids:
            if b1 != b2 and visible(b1, b2):
                visible_boids.append(b2)

        alignment_boids = []
        for b2 in visible_boids:
            if close_enough(b1, b2, alignment_r):
                alignment_boids.append(b2)

        cohesion_boids = []
        for b2 in alignment_boids:
            if close_enough(b1, b2, cohesion_r):
                cohesion_boids.append(b2)

        separation_boids = []
        for b2 in cohesion_boids:
            if close_enough(b1, b2, separation_r):
                separation_boids.append(b2)

        align(b1, visible_boids)
        cohese(b1, cohesion_boids)
        separate(b1, separation_boids)
        avoid_mouse(b1)
        avoid_edges(b1)
        add_noise(b1)

        b1.pos_x += b1.dir_x
        b1.pos_y += b1.dir_y

    mouse_outline.x = mouse_x
    mouse_outline.y = mouse_y

pg.clock.schedule_interval(update, update_dt)
pg.app.run()
