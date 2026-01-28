#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

def plot_control_analysis(csv_path):
    df = pd.read_csv(csv_path)
    t = df['timestamp'] - df['timestamp'].iloc[0]
    
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    
    # 1. Lateral error
    axes[0,0].plot(t, df['lat_err'] * 100, 'b-', linewidth=1.5)
    axes[0,0].axhline(0, color='k', linestyle='--', alpha=0.3)
    axes[0,0].set_ylabel('Lateral Error (cm)')
    axes[0,0].set_title('Lateral Tracking')
    axes[0,0].grid(True, alpha=0.3)
    
    # 2. Heading error
    axes[0,1].plot(t, np.degrees(df['hdg_err']), 'r-', linewidth=1.5)
    axes[0,1].set_ylabel('Heading Error (deg)')
    axes[0,1].set_title('Heading Tracking')
    axes[0,1].grid(True, alpha=0.3)
    
    # 3. Steering
    axes[1,0].plot(t, np.degrees(df['steer_angle']), 'g-', linewidth=1.5)
    axes[1,0].set_ylabel('Steering (deg)')
    axes[1,0].set_title('Steering Command')
    axes[1,0].grid(True, alpha=0.3)
    
    # 4. Motor
    axes[1,1].plot(t, df['motor_level'], 'm-', linewidth=1.5)
    axes[1,1].set_ylabel('Motor Level')
    axes[1,1].set_title('Throttle/Brake')
    axes[1,1].grid(True, alpha=0.3)
    
    # 5. Curvature
    axes[2,0].plot(t, df['curvature'], 'b-', linewidth=1.5)
    axes[2,0].set_ylabel('Curvature (1/m)')
    axes[2,0].set_xlabel('Time (s)')
    axes[2,0].set_title('Path Curvature')
    axes[2,0].grid(True, alpha=0.3)
    
    # 6. Errors
    axes[2,1].plot(t, np.abs(df['lat_err'] * 100), 'b-', label='|Lat|')
    axes[2,1].set_ylabel('Abs Error')
    axes[2,1].set_xlabel('Time (s)')
    axes[2,1].legend()
    axes[2,1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    csv_path = sys.argv[1] if len(sys.argv) > 1 else 'data/control_log.csv'
    plot_control_analysis(csv_path)