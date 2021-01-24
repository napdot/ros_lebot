
def thrower_calculation(basket_distance):
    trange = [0, 100]
    ser = int(basket_distance * 0.01352 + 18.79578)
    if ser < trange[0]:
        ser = int(23)
    if ser > trange[1]:
        ser = int(99)
    return ser
