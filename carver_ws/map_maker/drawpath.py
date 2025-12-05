import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy.interpolate import splprep, splev
import open3d as o3d
import os

pcd = o3d.io.read_point_cloud("pcd/GlobalMap.pcd")
points_3d = np.asarray(pcd.points)

waypoints = []
temp_point = None
temp_arrow = None

def onpress(event):
    global temp_point, temp_arrow
    if event.button == 1 and event.xdata and event.ydata:
        temp_point = [event.xdata, event.ydata]
        if temp_arrow:
            temp_arrow.remove()
            temp_arrow = None
        plt.draw()

def onrelease(event):
    global temp_point, temp_arrow
    if event.button == 1 and temp_point and event.xdata and event.ydata:
        dx = event.xdata - temp_point[0]
        dy = event.ydata - temp_point[1]
        yaw = np.arctan2(dy, dx)
        
        waypoints.append({'x': temp_point[0], 'y': temp_point[1], 'yaw': yaw})
        
        plt.clf()
        plt.scatter(points_3d[:, 0], points_3d[:, 1], c='gray', s=0.1, alpha=0.3)
        
        if len(waypoints) > 0:
            for i, wp in enumerate(waypoints):
                plt.scatter(wp['x'], wp['y'], c='red', s=100, zorder=5)
                plt.text(wp['x'], wp['y'], f' {i+1}', fontsize=12)
                
                arrow_length = 0.8
                dx_arrow = arrow_length * np.cos(wp['yaw'])
                dy_arrow = arrow_length * np.sin(wp['yaw'])
                plt.arrow(wp['x'], wp['y'], dx_arrow, dy_arrow,
                         head_width=0.3, head_length=0.3, fc='blue', ec='blue', alpha=0.7)
        
        if len(waypoints) >= 2:
            wp_x = [wp['x'] for wp in waypoints]
            wp_y = [wp['y'] for wp in waypoints]
            wp_x.append(waypoints[0]['x'])
            wp_y.append(waypoints[0]['y'])
            
            if len(waypoints) >= 3:
                tck, u = splprep([wp_x, wp_y], s=0.5, per=False)
                u_new = np.linspace(0, 1, 100)
                smooth_x, smooth_y = splev(u_new, tck)
                plt.plot(smooth_x, smooth_y, 'g-', linewidth=2, label='Smooth path')
            else:
                plt.plot(wp_x, wp_y, 'g-', linewidth=2)
        
        plt.axis('equal')
        plt.grid(True, alpha=0.3)
        plt.title(f'Click and drag to add waypoints ({len(waypoints)} points)')
        plt.draw()
        
        temp_point = None
        temp_arrow = None

def onmotion(event):
    global temp_point, temp_arrow
    if temp_point and event.xdata and event.ydata:
        if temp_arrow:
            temp_arrow.remove()
        
        dx = event.xdata - temp_point[0]
        dy = event.ydata - temp_point[1]
        temp_arrow = plt.arrow(temp_point[0], temp_point[1], dx, dy,
                              head_width=0.3, head_length=0.3, 
                              fc='orange', ec='orange', alpha=0.5, linestyle='--')
        plt.draw()

fig, ax = plt.subplots(figsize=(12, 10))
plt.scatter(points_3d[:, 0], points_3d[:, 1], c='gray', s=0.1, alpha=0.3)
plt.axis('equal')
plt.grid(True, alpha=0.3)
plt.title('Click and drag to add waypoints (0 points)')

fig.canvas.mpl_connect('button_press_event', onpress)
fig.canvas.mpl_connect('button_release_event', onrelease)
fig.canvas.mpl_connect('motion_notify_event', onmotion)
plt.show()

if len(waypoints) >= 2:
    os.makedirs('output', exist_ok=True)
    
    wp_x = [wp['x'] for wp in waypoints]
    wp_y = [wp['y'] for wp in waypoints]
    wp_x_closed = wp_x + [waypoints[0]['x']]
    wp_y_closed = wp_y + [waypoints[0]['y']]
    
    path_data = []
    if len(waypoints) >= 3:
        tck, u = splprep([wp_x_closed, wp_y_closed], s=0.5, per=False)
        u_new = np.linspace(0, 1, 100)
        smooth_x, smooth_y = splev(u_new, tck)
        
        for i in range(len(smooth_x) - 1):
            dx = smooth_x[i + 1] - smooth_x[i]
            dy = smooth_y[i + 1] - smooth_y[i]
            yaw = np.arctan2(dy, dx)
            
            path_data.append({
                'x': float(smooth_x[i]),
                'y': float(smooth_y[i]),
                'yaw': float(yaw)
            })
    else:
        for wp in waypoints:
            path_data.append({
                'x': float(wp['x']),
                'y': float(wp['y']),
                'yaw': float(wp['yaw'])
            })
    
    with open('output/path.yaml', 'w') as f:
        yaml.dump(path_data, f, default_flow_style=False)
    
    fig_save = plt.figure(figsize=(12, 10))
    plt.scatter(points_3d[:, 0], points_3d[:, 1], c='gray', s=0.1, alpha=0.3, label='Point Cloud')
    
    for i, wp in enumerate(waypoints):
        plt.scatter(wp['x'], wp['y'], c='red', s=100, zorder=5)
        plt.text(wp['x'], wp['y'], f' {i+1}', fontsize=12, fontweight='bold')
        
        arrow_length = 0.8
        dx = arrow_length * np.cos(wp['yaw'])
        dy = arrow_length * np.sin(wp['yaw'])
        plt.arrow(wp['x'], wp['y'], dx, dy,
                 head_width=0.3, head_length=0.3, fc='blue', ec='blue', alpha=0.7)
    
    if len(waypoints) >= 3:
        tck, u = splprep([wp_x_closed, wp_y_closed], s=0.5, per=False)
        u_new = np.linspace(0, 1, 100)
        smooth_x, smooth_y = splev(u_new, tck)
        plt.plot(smooth_x, smooth_y, 'g-', linewidth=2, label='Smooth path')
    else:
        plt.plot(wp_x_closed, wp_y_closed, 'g-', linewidth=2, label='Path')
    
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.title(f'Path with {len(waypoints)} waypoints')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.savefig('output/path.png', dpi=300, bbox_inches='tight')
    plt.close(fig_save)
    
    print(f"Saved {len(path_data)} waypoints to output/path.yaml")
    print("Saved path visualization to output/path.png")
else:
    print("Need at least 2 points to create a path")