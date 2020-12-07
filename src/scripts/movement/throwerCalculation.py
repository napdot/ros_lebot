
def thrower_calculation(basket_distance):
    max_serial = 190
    km = 300
    thrower_speed = (max_serial * basket_distance) / (km + basket_distance)
    if thrower_speed < 190:
        return thrower_speed
    else:
        return 100

