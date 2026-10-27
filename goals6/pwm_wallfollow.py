import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# DriveSystem motions mapping from style to (left_pwm, right_pwm)
motions = {
    'TURN_LEFT':   (0.6, 1), 
    'STEER_LEFT':  (0.79, 0.97), 
    'VEER_LEFT':   (0.80, 0.92), 
    'STRAIGHT':    (0.81, 0.79), 
    'VEER_RIGHT':  (0.94, 0.79),
    'STEER_RIGHT': (0.99, 0.74), 
    'TURN_RIGHT':  (1, 0.51)
}


# Representative error points for each discrete action band
band = 0.10 / 3.0
error_points = [
    -2*band - 0.01,      # TURN_LEFT region (~ -0.077)
    -1*band - (band/2),  # STEER_LEFT (~ -0.05)
    -band/2,             # VEER_LEFT (~ -0.0167)
    band/2,              # STRAIGHT (~ 0.0167)
    band + (band/2),     # VEER_RIGHT (~ 0.05)
    2*band + (band/2),   # STEER_RIGHT (~ 0.0833)
    3*band               # TURN_RIGHT (~ 0.10)
]

styles = [
    'TURN_LEFT', 'STEER_LEFT', 'VEER_LEFT',
    'STRAIGHT', 'VEER_RIGHT', 'STEER_RIGHT', 'TURN_RIGHT'
]

data = []
for e, style in zip(error_points, styles):
    l, r = motions[style]
    data.append({'error': e, 'style': style, 'left_pwm': l, 'right_pwm': r})
df = pd.DataFrame(data)

from ace_tools import display_dataframe_to_user
display_dataframe_to_user("PWM vs Error Bands", df)

coeffs_l = np.polyfit(df['error'], df['left_pwm'], 1)
coeffs_r = np.polyfit(df['error'], df['right_pwm'], 1)
kl, cl = coeffs_l[0], coeffs_l[1]
kr, cr = coeffs_r[0], coeffs_r[1]

# Plot Left PWM vs Error
plt.figure()
plt.plot(df['error'], df['left_pwm'], marker='o')
plt.plot(df['error'], cl + kl * df['error'])
plt.xlabel('Error (m)')
plt.ylabel('Left PWM')
plt.title('Left PWM vs Distance Error')
plt.show()

# Plot Right PWM vs Error
plt.figure()
plt.plot(df['error'], df['right_pwm'], marker='o')
plt.plot(df['error'], cr + kr * df['error'])
plt.xlabel('Error (m)')
plt.ylabel('Right PWM')
plt.title('Right PWM vs Distance Error')
plt.show()

print(f"Left PWM ≈ {cl:.3f} + ({kl:.3f}) × error")
print(f"Right PWM ≈ {cr:.3f} + ({kr:.3f}) × error")
