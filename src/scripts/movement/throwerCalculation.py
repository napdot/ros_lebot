
def thrower_calculation(basket_distance):
    trange = [1390, 2020]
    ser = int(basket_distance * 0.208045 + 1127.105967)
    if ser < trange[0]:
        ser = 1390
    if ser > trange[1]:
        ser = 2020
    return ser
