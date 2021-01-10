
def thrower_calculation(basket_distance):
    trange = [1390, 2000]
    ser = int(basket_distance * 0.208045 + 1117.105967)
    if ser < trange[0]:
        ser = 1390
    if ser > trange[1]:
        ser = 2000
    return ser
