import math
from constants import *
import pybullet as p # type: ignore
import numpy as np
import time

DEFAULT_COMPUTE_IK_SIGN = -1

# Fonction d'Al-Kashi
def alKashi(a, b, c, sign=1):
    if a * b == 0: return 0
    # Note : pour obtenir l'autre solution, il suffit de changer le signe du retour
    return sign * math.acos(min(1, max(-1, (a**2 + b**2 - c**2) / (2 * a * b))))

def computeDK( theta1, theta2, theta3, L1=constL1, L2=constL2, L3=constL3, use_rads=USE_RADS_INPUT, use_mm=USE_MM_OUTPUT):

    angle_unit = 1
    dist_unit = 1

    if not (use_rads): angle_unit = math.pi / 180.0
    if use_mm: dist_unit = 1000

    # Conversion des angles en radians si nécessaire
    theta1 = THETA1_MOTOR_SIGN * theta1 * angle_unit
    theta2 = (THETA2_MOTOR_SIGN * theta2 - theta2Correction) * angle_unit
    theta3 = (THETA3_MOTOR_SIGN * theta3 - theta3Correction) * angle_unit

    planContribution = L1 + L2 * math.cos(theta2) + L3 * math.cos(theta2 + theta3)

    # Calcul des coordonnées x, y, z en fonction des angles et distances
    x = math.cos(theta1) * planContribution * dist_unit
    y = math.sin(theta1) * planContribution * dist_unit
    z = -(L2 * math.sin(theta2) + L3 * math.sin(theta2 + theta3)) * dist_unit

    return [x, y, z]

# Fonction cinématique directe
def computeDKDetailed(theta1, theta2, theta3, L1=constL1, L2=constL2, L3=constL3, use_rads=USE_RADS_INPUT, use_mm=USE_MM_OUTPUT):

    theta1_verif = theta1
    theta2_verif = theta2
    theta3_verif = theta3
    angle_unit = 1
    dist_unit = 1

    if not (use_rads): angle_unit = math.pi / 180.0
    if use_mm: dist_unit = 1000

    # Conversion des angles en radians si nécessaire
    theta1 = THETA1_MOTOR_SIGN * theta1 * angle_unit
    theta2 = (THETA2_MOTOR_SIGN * theta2 - theta2Correction) * angle_unit
    theta3 = (THETA3_MOTOR_SIGN * theta3 - theta3Correction) * angle_unit

    planContribution = L1 + L2 * math.cos(theta2) + L3 * math.cos(theta2 + theta3)

    # Calcul des coordonnées x, y, z en fonction des angles et distances
    x = math.cos(theta1) * planContribution
    y = math.sin(theta1) * planContribution
    z = -(L2 * math.sin(theta2) + L3 * math.sin(theta2 + theta3))

    # Calcul des positions intermédiaires pour la vérification
    p0 = [0, 0, 0]
    p1 = [L1 * math.cos(theta1) * dist_unit, L1 * math.sin(theta1) * dist_unit, 0]
    p2 = [
        (L1 + L2 * math.cos(theta2)) * math.cos(theta1) * dist_unit,
        (L1 + L2 * math.cos(theta2)) * math.sin(theta1) * dist_unit,
        -L2 * math.sin(theta2) * dist_unit,
    ]
    p3 = [x * dist_unit, y * dist_unit, z * dist_unit]

    p3_verif = computeDK(theta1_verif, theta2_verif, theta3_verif, L1, L2, L3, use_rads, use_mm)

    if (p3[0] != p3_verif[0]) or (p3[1] != p3_verif[1]) or (p3[2] != p3_verif[2]):
        print(
            "ERREUR: la fonction DK est cassée!!! p3 = {}, p3_verif = {}".format(
                p3, p3_verif
            )
        )

    return [p0, p1, p2, p3]

# Fonction cinématique inverse
def computeIK( x, y, z, L1=constL1, L2=constL2, L3=constL3, verbose=False, use_rads=USE_RADS_OUTPUT, sign=DEFAULT_COMPUTE_IK_SIGN, use_mm=USE_MM_INPUT):

    dist_unit = 1
    if use_mm: dist_unit = 0.001

    x = x * dist_unit
    y = y * dist_unit
    z = z * dist_unit

    # Calcul de theta1 en fonction de la position dans le plan X/Y
    if y == 0 and x == 0: theta1 = 0
    else: theta1 = math.atan2(y, x)

    # Calcul des distances entre les moteurs et la fin de la jambe
    xp = math.sqrt(x * x + y * y) - L1
    d = math.sqrt(math.pow(xp, 2) + math.pow(z, 2))

    # Calcul de theta2 et theta3 avec la loi d'Al Kashi
    theta2 = alKashi(L2, d, L3, sign=sign) - Z_DIRECTION * math.atan2(z, xp)
    theta3 = math.pi + alKashi(L2, L3, d, sign=sign)

    # Conversion en radians ou degrés selon l'option
    if use_rads:
        result = [
            angleRestrict(THETA1_MOTOR_SIGN * theta1, use_rads=use_rads),
            angleRestrict(THETA2_MOTOR_SIGN * (theta2 + theta2Correction), use_rads=use_rads),
            angleRestrict(THETA3_MOTOR_SIGN * (theta3 + theta3Correction), use_rads=use_rads),
        ]
    else:
        result = [
            angleRestrict(THETA1_MOTOR_SIGN * math.degrees(theta1), use_rads=use_rads),
            angleRestrict(THETA2_MOTOR_SIGN * (math.degrees(theta2) + theta2Correction), use_rads=use_rads,),
            angleRestrict(THETA3_MOTOR_SIGN * (math.degrees(theta3) + theta3Correction), use_rads=use_rads,),
        ]
    if verbose:
        print(
            "Demande de cinématique inverse pour x={}, y={}, z={}\n --> theta1={}, theta2={}, theta3={}".format(
                x, y, z, result[0], result[1], result[2],
            )
        )

    return result

def modulo180(angle):
    if -180 < angle < 180: return angle

    angle = angle % 360
    if angle > 180: return -360 + angle

    return angle

def modulopi(angle):
    if -math.pi < angle < math.pi: return angle

    angle = angle % (math.pi * 2)
    if angle > math.pi: return -math.pi * 2 + angle

    return angle

def angleRestrict(angle, use_rads=False):
    if use_rads: return modulopi(angle)
    else: return modulo180(angle)

# Fonction Circle de face
def circle_face(x, y, r, t, duration, leg_id=None):
    angle = (t % duration) / duration * 2 * math.pi
    target_x = x
    target_y = y + r * math.cos(angle)
    target_z = r * math.sin(angle)  # Adjusted to make the circle perpendicular to the ground
    
    if leg_id is None:
        alpha = computeIK(target_x, target_y, target_z)
    else:
        alpha = computeIKOriented(target_x, target_y, target_z, leg_id, angle)
    
    return alpha

# Fonction Circle
def circle(x, z, r,t, duration):
    y = 0
    time = t / duration * 2
    x = x + r * math.cos(time)
    z = z + r * math.sin(time)

    return computeIK(x, y, z)

# Fonction Segment
def segment(segment_x1, segment_y1, segment_z1, segment_x2, segment_y2, segment_z2, t, duration,):

    t_cycle = t % (2 * duration)

    if t_cycle > duration: t_effective = 2 * duration - t_cycle
    else: t_effective = t_cycle

    fraction = t_effective / duration
    xm = segment_x1 + fraction * (segment_x2 - segment_x1)
    ym = segment_y1 + fraction * (segment_y2 - segment_y1)
    zm = segment_z1 + fraction * (segment_z2 - segment_z1)

    return [xm, ym, zm]

# Fonction rotation 2D autour de l'axe Z
def rotaton_2D(x, y, z, angle):

    x_rot = x * math.cos(angle) - y * math.sin(angle)
    y_rot = x * math.sin(angle) + y * math.cos(angle)
    z_rot = z

    return [x_rot, y_rot, z_rot]

# Fonction pour calculer la cinématique inverse orientée avec un angle spécifique
def computeIKOriented(x, y, z, leg_id, angle=None):
    if angle is None: angle = 0

    offset_x = 0.2
    offset_y = 0
    offset_z = -0.05
    new_x, new_y, new_z = rotaton_2D(x, y, z, -LEG_ANGLES[leg_id - 1] + angle)

    return computeIK(new_x + offset_x, new_y + offset_y, new_z + offset_z)

# Fonction pour récupérer un angle supplémentaire en fonction des touches du clavier
def get_extra_angle():
    keys = p.getKeyboardEvents()
    if ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
        return 0
    if ord('q') in keys and keys[ord('q')] & p.KEY_IS_DOWN:
        return -math.pi
    if ord('z') in keys and keys[ord('z')] & p.KEY_IS_DOWN:
        return math.pi / 2
    if ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN:
        return -math.pi / 2
    return None

# Fonction d'interpolation entre deux points selon un ratio
def interpol(p1, p2, ratio):
    return p1 + ratio * (p2 - p1)

# Fonction qui effectue un mouvement en forme de triangle pour la jambe
def triangle_motion(x, z, h, w, t, sequence, leg_id=None):
    extra_angle = get_extra_angle()
    phase = t % 2

    if phase < 1:
        ratio = phase
        y1, y2, z1, z2 = sequence[0]
    elif phase < 1.5:
        ratio = (phase - 1) * 2
        y1, y2, z1, z2 = sequence[1]
    else:
        ratio = (phase - 1.5) * 2
        y1, y2, z1, z2 = sequence[2]

    target_y = interpol(y1, y2, ratio)
    target_z = interpol(z1, z2, ratio)

    if leg_id is None:
        return computeIK(x, target_y, target_z)
    else:
        return computeIKOriented(x, target_y, target_z, leg_id, extra_angle)

# Fonction triangl 
def triangle(x, z, h, w, t, leg_id=None):
    sequence = [
        (w/2, -w/2, z, z),
        (-w/2, 0, z, z + h),
        (0, w/2, z + h, z)
    ]
    return triangle_motion(x, z, h, w, t, sequence, leg_id)

# Fonction triangle
def triangle2(x, z, h, w, t, leg_id=None):
    sequence = [
        (-w/2, w/2, z, z),
        (w/2, 0, z, z + h),
        (0, -w/2, z + h, z)
    ]
    return triangle_motion(x, z, h, w, t, sequence, leg_id)

def goto_position(sim, robot, target_position):
    # 2. Lire la position actuelle du robot
    pos, _ = p.getBasePositionAndOrientation(sim.robot)
    x, y, _ = pos
    target_x, target_y = target_position

    # 3. Calculer la distance entre la position actuelle et la position cible
    dx = target_x - x
    dy = target_y - y
    distance = math.sqrt(dx**2 + dy**2)

    if distance < 0.05:
        print(f"✅ Position atteinte : {target_position}")
        return

    # 4. Calculer l'angle vers la cible
    angle = math.atan2(dy, dx)

    # 5. Définir les directions comme les touches z/q/s/d
    directions = {
        0: 0,                         # droite (d)
        math.pi / 2: math.pi / 2,     # haut (z)
        -math.pi / 2: -math.pi / 2,   # bas (s)
        -math.pi: -math.pi            # gauche (q)
    }

    # 6. Trouver la direction la plus proche
    closest = min(directions.keys(), key=lambda a: abs((a - angle + math.pi) % (2 * math.pi) - math.pi))
    extra_angle = closest

    # 7. Déplacer lentement vers la cible (en petits pas)
    step_size = 0.001  # petit pas pour laisser le temps de l'animation
    new_x = x + step_size * math.cos(extra_angle)
    new_y = y + step_size * math.sin(extra_angle)
    p.resetBasePositionAndOrientation(sim.robot, [new_x, new_y, pos[2]], [0, 0, 0, 1])

    # 8. Animer les pattes du robot pour simuler le mouvement
    for l in [1, 3, 5]:  # pour les jambes 1, 3, 5
        thetas = triangle(extra_angle, -0.05, 0.03, 0.08, sim.t, leg_id=l)
        for m in range(3):
            robot.legs[l][m].goal_position = thetas[m]
    
    for l in [2, 4, 6]:  # pour les jambes 2, 4, 6
        thetas = triangle(extra_angle, -0.05, 0.03, 0.08, sim.t + 1, leg_id=l)
        for m in range(3):
            robot.legs[l][m].goal_position = thetas[m]
        