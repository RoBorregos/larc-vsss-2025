import numpy as np
from matplotlib.path import Path
import matplotlib.pyplot as plt
import cv2

def generar_plano_uv(y_value=128):
    U = np.tile(np.arange(0, 256), (256, 1)).astype(np.uint8)
    V = np.tile(np.arange(0, 256).reshape(-1, 1), (1, 256)).astype(np.uint8)
    Y = np.full((256, 256), y_value, dtype=np.uint8)
    yuv_image = cv2.merge([Y, U, V])
    bgr_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2RGB)
    return bgr_image

def allPoints(pts:list) -> list: 
    polygon_uv = np.array(pts)
    polygon_path = Path(polygon_uv)

    u_range = np.arange(0, 256)
    v_range = np.arange(0, 256)
    uu, vv = np.meshgrid(u_range, v_range)
    uv_grid = np.vstack((uu.flatten(), vv.flatten())).T

    mask_inside = polygon_path.contains_points(uv_grid)
    uv_inside = uv_grid[mask_inside]

    print(f"{len(uv_inside)} puntos UV dentro del polígono")
    return uv_inside

def realPoints(polygon_points, u_points, v_points):
    path = Path(polygon_points)
    uv_points = np.column_stack((u_points, v_points))
    mask = path.contains_points(uv_points)
    print(f"{len(uv_points[mask])} puntos UV de la imagen dentro del polígono")
    return uv_points[mask]

def pointsSelection(u_points, v_points, color, colors_rgb=None):
    points = []

    plano_uv = generar_plano_uv(y_value=128)

    fig, ax = plt.subplots(figsize=(10,7))
    ax.imshow(plano_uv)
    ax.set_title("Selecting borders for {} color".format(color))
    ax.set_xlim(0, 255)
    ax.set_ylim(0, 255)
    ax.set_xlabel("U")
    ax.set_ylabel("V")
    ax.invert_yaxis()
    plt.grid(True)

    if colors_rgb is not None:
        ax.scatter(u_points, v_points, c=colors_rgb, s=1)
    else:
        ax.scatter(u_points, v_points, c='black', edgecolors='k', s=1, linewidths=0.5)

    line, = ax.plot([], [], 'ro-', lw=2)

    def on_click(event):
        if event.button == 1 and event.inaxes:
            points.append((event.xdata, event.ydata))
            xs, ys = zip(*points)
            line.set_data(xs, ys)
            fig.canvas.draw()

    def on_key(event):
        if event.key == 'enter' and len(points) >= 3:
            plt.close()

    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.show()

    return points

def create_lut(uv_points):
    lut = np.zeros((256, 256), dtype=np.uint8)

    for u, v in uv_points:
        if 0 <= u < 256 and 0 <= v < 256:
            lut[v, u] = 1  

    return lut


img_paths = [
    "Resources/blue/19.png",
    "Resources/blue/20.png",
    "Resources/blue/28.png",
    "Resources/blue/29.png",
    "Resources/blue/30.png",
    "Resources/blue/31.png"
]
colors = ["blue"]


for color in colors: 
    u_points = []
    v_points = []
    colors_rgb = []

    for img in img_paths:
        img = cv2.imread(img)
        img = cv2.resize(img, (500, 500))
        img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)

        u = img_yuv[:, :, 1].flatten()
        v = img_yuv[:, :, 2].flatten()
        rgb = img.reshape(-1, 3)[:, ::-1] / 255.0  # BGR a RGB

        u_points.extend(u)
        v_points.extend(v)
        colors_rgb.extend(rgb)

    points = pointsSelection(u_points, v_points, color, colors_rgb)
    all_points = allPoints(points)
    
    lut = create_lut(all_points)
    name = "lut_" + color
    np.save(name, lut)

    print("LUTs for {} color saved".format(color))