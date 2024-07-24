import math

def calc_hipo_angle(screen_rat_x_y, x_y, alt, yaw, alt_met):
    screen_x = screen_rat_x_y[0]
    screen_y = screen_rat_x_y[1]
    screen_y_mid = screen_y/2
    screen_x_mid = screen_x/2
    x = x_y[0]
    y = x_y[1]

    hipo = math.sqrt(abs(screen_x_mid-x)**2 + abs(screen_y_mid-y)**2)

    angle = math.degrees(math.atan2(abs(screen_x_mid-x),abs(screen_y_mid-y)))

    x_sign = x-screen_x_mid
    y_sign = y-screen_y_mid

    if x_sign < 0 and y_sign < 0:
        angle = 270+angle
    elif x_sign < 0 and y_sign > 0:
        angle = 180 + angle
    elif x_sign > 0 and y_sign < 0:
        angle = angle
    elif x_sign > 0 and y_sign > 0:
        angle = 180 - angle
    elif x_sign == 0 and y_sign < 0:
        angle = 0
    elif x_sign == 0 and y_sign > 0:
        angle = 180
    elif x_sign < 0 and y_sign == 0:
        angle = 270
    elif x_sign > 0 and y_sign == 0:
        angle = 90
    elif x_sign == 0 and y_sign == 0:
        angle = 0

    #       metre       derece
    return hipo*alt/alt_met, (yaw + angle) % 360