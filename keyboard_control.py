import keyboard

def get_keyboard_inputs():
    steer = 0
    throttle = 0
    brake = 0

    if keyboard.is_pressed("left"):
        steer = -1  # Full left
    elif keyboard.is_pressed("right"):
        steer = 1  # Full right

    if keyboard.is_pressed("up"):
        throttle = 1  # Accelerate
    elif keyboard.is_pressed("down"):
        brake = 1  # Brake

    return steer, throttle, brake

st,th,br=get_keyboard_inputs()
