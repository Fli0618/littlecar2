import os
import cv2
import numpy as np
import random
from PIL import Image, ImageDraw, ImageFont

# ==================== 配置区域 ====================
NUM_IMAGES = 1500              # 生成图像的总数量
SAVE_DIR = os.path.join("assets", "circle_with_number_v2")  # 导出保存路径
IMAGE_SIZE = (640, 640)         # 生成图像的分辨率 (宽, 高)
TRAIN_RATIO = 0.7               # 训练集与验证集的比例 (7:3)
IDEAL_MODE_RATIO = 0.15         # 极佳环境（理想无噪点模式）的比例
HIGHLIGHT_PROB = 0.35           # 高光开启概率

# 缩放倍率
MIN_SCALE = 0.60                
MAX_SCALE = 2.00                

# 局部高斯模糊配置
REGIONAL_BLUR_PROB = 0.40       
BLUR_GRID_DIV = 4               
MIN_BLUR_KERNEL = 15            
MAX_BLUR_KERNEL = 45            

# ==================== 同心圆环物理参数 ====================
PHI = 50.0
W_TRACK = 1.5
G_OUTER = 3.2
N_TEETH = 24
TOOTH_RATIO = 2.0 / 3.0
W_LINE_RATIO = 1.0 / 3.0

# 预计算半径
track_radii = []
r_in_1 = (PHI + 3.0) / 2.0
r_out_1 = r_in_1 + W_TRACK
track_radii.append((r_in_1, r_out_1))

r_in_2 = r_in_1 + 2.5
r_out_2 = r_in_2 + W_TRACK
track_radii.append((r_in_2, r_out_2))

r_in_3 = r_in_2 + 3.5
r_out_3 = r_in_3 + W_TRACK
track_radii.append((r_in_3, r_out_3))

r_in_4 = r_out_3 + G_OUTER
r_out_4 = r_in_4 + W_TRACK
track_radii.append((r_in_4, r_out_4))

r_in_5 = r_out_4 + G_OUTER
r_out_5 = r_in_5 + W_TRACK
track_radii.append((r_in_5, r_out_5))

r_in_6 = r_out_5 + G_OUTER
r_out_6 = r_in_6 + W_TRACK
track_radii.append((r_in_6, r_out_6))

tracks_config = [
    {"type": "solid"},
    {"type": "serrated"},
    {"type": "solid"},
    {"type": "serrated"},
    {"type": "solid"},
    {"type": "serrated"}
]

period_deg = 360.0 / N_TEETH
tooth_deg = period_deg * TOOTH_RATIO
w_line = W_TRACK * W_LINE_RATIO

# 类别映射 (1->0, 2->1, 3->2)
CLASS_MAP = {
    0: "1",
    1: "2",
    2: "3"
}


# ==================== 基础与环境仿真函数 ====================

def save_image_robust(img, file_path):
    """支持中文路径的安全图像保存"""
    try:
        dir_name = os.path.dirname(file_path)
        os.makedirs(dir_name, exist_ok=True)
        is_success, im_buf_arr = cv2.imencode(".jpg", img)
        if is_success:
            im_buf_arr.tofile(file_path)
    except Exception as e:
        print(f"保存文件出错 {file_path}: {e}")


def draw_irregular_shadow_set(mask, width, height, angle_rad):
    """在指定的掩膜上沿特定角度绘制一组平行条带阴影"""
    cx, cy = width // 2, height // 2
    perp_angle = angle_rad + np.pi / 2
    dx, dy = np.cos(perp_angle), np.sin(perp_angle)
    
    start_dist = -int(max(width, height) * 1.0)
    end_dist = int(max(width, height) * 1.0)
    curr_dist = start_dist
    lx, ly = np.cos(angle_rad), np.sin(angle_rad)
    length = max(width, height) * 3.5
    
    while curr_dist < end_dist:
        curr_band_width = random.choice([
            random.randint(15, 35),
            random.randint(40, 80),
            random.randint(95, 165)
        ])
        px = cx + dx * curr_dist
        py = cy + dy * curr_dist
        p1 = (int(px - lx * length), int(py - ly * length))
        p2 = (int(px + lx * length), int(py + ly * length))
        intensity = random.randint(160, 255)
        cv2.line(mask, p1, p2, intensity, curr_band_width)
        gap = random.randint(30, 240)
        curr_dist += curr_band_width + gap


def generate_white_harsh_background(width, height, ideal_mode=False, limit_shadows=True):
    """生成包含线性渐变、不规则条带阴影与暗角的白/灰度背景"""
    if ideal_mode:
        return np.ones((height, width, 3), dtype=np.uint8) * 245

    base_white = random.randint(238, 246)
    bg = np.ones((height, width, 3), dtype=np.uint8) * base_white

    # 线性渐变
    grad_map = np.ones((height, width), dtype=np.float32)
    gradient_direction = random.choice(["top_bottom", "left_right", "diagonal"])
    grad_intensity = random.uniform(0.15, 0.35)
    
    if gradient_direction == "top_bottom":
        for y in range(height):
            grad_map[y, :] = 1.0 - (y / height) * grad_intensity
    elif gradient_direction == "left_right":
        for x in range(width):
            grad_map[:, x] = 1.0 - (x / width) * grad_intensity
    else:
        for y in range(height):
            for x in range(width):
                grad_map[y, x] = 1.0 - ((x + y) / (width + height)) * grad_intensity

    for c in range(3):
        bg[:, :, c] = (bg[:, :, c] * grad_map).astype(np.uint8)

    # 阴影投影
    mask1 = np.zeros((height, width), dtype=np.uint8)
    angle1 = np.radians(random.uniform(-55, 55))
    draw_irregular_shadow_set(mask1, width, height, angle1)

    mask2 = np.zeros((height, width), dtype=np.uint8)
    angle_offset = random.choice([random.uniform(40, 85), random.uniform(-85, -40)])
    angle2 = angle1 + np.radians(angle_offset)
    draw_irregular_shadow_set(mask2, width, height, angle2)

    shadow_mask = cv2.addWeighted(mask1, 0.65, mask2, 0.65, 0)
    blur_k = random.choice([85, 105, 125])
    shadow_mask_blurred = cv2.GaussianBlur(shadow_mask, (blur_k, blur_k), 0)
    
    shadow_opacity = random.uniform(0.12, 0.22) if limit_shadows else random.uniform(0.25, 0.40)
    alpha = (shadow_mask_blurred / 255.0) * shadow_opacity
    
    for c in range(3):
        target_shadow_color = 30
        bg[:, :, c] = (bg[:, :, c] * (1.0 - alpha) + target_shadow_color * alpha).astype(np.uint8)

    # 镜头暗角
    X, Y = np.meshgrid(np.arange(width), np.arange(height))
    dist_from_center = np.sqrt((X - width / 2) ** 2 + (Y - height / 2) ** 2)
    max_dist = np.sqrt((width / 2) ** 2 + (height / 2) ** 2)
    vignette_drop = random.uniform(0.08, 0.20) if limit_shadows else random.uniform(0.18, 0.35)
    vignette = 1.0 - vignette_drop * (dist_from_center / max_dist)
    for c in range(3):
        bg[:, :, c] = np.clip(bg[:, :, c] * vignette, 0, 255).astype(np.uint8)

    return bg


def draw_geometric_interference(img, width, height, forbidden_bgrs=None, draw_on_top=False):
    """在画布上叠加绘制随机的几何干涉线条（背景或前景干扰）"""
    temp_layer = img.copy()

    def get_safe_color():
        if not forbidden_bgrs:
            return (random.randint(120, 200), random.randint(120, 200), random.randint(120, 200))
        for _ in range(30):
            c = (random.randint(120, 200), random.randint(120, 200), random.randint(120, 200))
            if all(np.linalg.norm(np.array(c) - np.array(fb)) > 60.0 for fb in forbidden_bgrs):
                return c
        return (150, 150, 150)

    num_lines = random.randint(2, 4) if draw_on_top else random.randint(3, 6)
    for _ in range(num_lines):
        p1 = (random.randint(-50, width + 50), random.randint(-50, height + 50))
        p2 = (random.randint(-50, width + 50), random.randint(-50, height + 50))
        color = get_safe_color()
        thickness = random.randint(1, 2)
        cv2.line(temp_layer, p1, p2, color, thickness, cv2.LINE_AA)

    alpha = random.uniform(0.20, 0.45)
    cv2.addWeighted(temp_layer, alpha, img, 1 - alpha, 0, dst=img)


def apply_camera_noise_dynamic(img, noise_level):
    """增加多尺度图像噪声（颗粒噪点与胡椒斑点）"""
    if noise_level <= 0.01:
        return img
    height, width, _ = img.shape
    noise_sigma = noise_level * random.uniform(15.0, 35.0)
    noise = np.random.normal(0, noise_sigma, img.shape).astype(np.int16)
    img_noisy = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    prob = noise_level * random.uniform(0.01, 0.03)
    rnd = np.random.rand(height, width)
    img_noisy[rnd < prob * 0.5] = [15, 15, 15]
    img_noisy[rnd > 1 - prob * 0.5] = [240, 240, 240]
    return img_noisy


def apply_regional_blur(img, grid_div, min_k, max_k):
    """区域失焦高斯模糊处理"""
    height, width, _ = img.shape
    cell_h = height // grid_div
    cell_w = width // grid_div

    row = random.randint(0, grid_div - 1)
    col = random.randint(0, grid_div - 1)

    ymin = row * cell_h
    ymax = height if row == grid_div - 1 else (row + 1) * cell_h
    xmin = col * cell_w
    xmax = width if col == grid_div - 1 else (col + 1) * cell_w

    roi = img[ymin:ymax, xmin:xmax]
    k_size = random.randint(min(min_k, max_k), max(min_k, max_k))
    if k_size % 2 == 0:
        k_size = max(1, k_size + 1)
    
    blurred_roi = cv2.GaussianBlur(roi, (k_size, k_size), 0)
    img[ymin:ymax, xmin:xmax] = blurred_roi
    return img


# ==================== 精确中心对齐渲染核心函数 ====================

def draw_rotated_text_opencv(img, text, center, angle, font_size, color_bgr):
    """
    在中-中(mm)对齐的透明画布正中央绘制并旋转数字，确保字心与同心圆心在数学与视觉上保持重合。
    """
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    pil_img = Image.fromarray(img_rgb)
    
    font = None
    font_candidates = [
        "arialbd.ttf",          # Windows Arial Bold
        "Arial Bold.ttf",       # macOS Arial Bold
        "DejaVuSans-Bold.ttf",  # Linux Bold
        "LiberationSans-Bold.ttf",
        "SimHei.ttf",           # 中文黑体
        "STHeiti Bold.ttc"
    ]
    for font_name in font_candidates:
        try:
            font = ImageFont.truetype(font_name, int(font_size))
            break
        except IOError:
            continue
    if font is None:
        font = ImageFont.load_default()

    # 构建一个富余的对称正方形透明画布，保证旋转时字形不被裁剪
    canvas_size = int(font_size * 2.5)
    canvas_size = max(20, canvas_size)
    txt_img = Image.new("RGBA", (canvas_size, canvas_size), (0, 0, 0, 0))
    txt_draw = ImageDraw.Draw(txt_img)
    
    color_rgb = (color_bgr[2], color_bgr[1], color_bgr[0], 255)
    mid_x = canvas_size / 2.0
    mid_y = canvas_size / 2.0
    
    # 强制使文本基于其中心点(mm)定位在画布正中央
    try:
        txt_draw.text((mid_x, mid_y), text, font=font, fill=color_rgb, anchor="mm")
    except TypeError:
        # 兼容旧版本 Pillow
        if hasattr(font, 'getbbox'):
            bbox = font.getbbox(text)
            w, h = bbox[2] - bbox[0], bbox[3] - bbox[1]
        else:
            w, h = font.getsize(text)
        tx = mid_x - w / 2.0
        ty = mid_y - h / 2.0
        txt_draw.text((tx, ty), text, font=font, fill=color_rgb)
    
    # 绕画布正中心进行高质量插值旋转 (expand=True 保证画布自动扩张但中心不变)
    rotated_txt = txt_img.rotate(angle, resample=Image.BICUBIC, expand=True)
    
    # 将旋转展开后的新画布几何中心对齐到传入的 center
    rw, rh = rotated_txt.size
    px = int(center[0] - rw // 2)
    py = int(center[1] - rh // 2)
    
    pil_img.paste(rotated_txt, (px, py), rotated_txt)
    return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)


def draw_encoder_pattern(img, center, target_number, scale, ideal_mode=False, highlight_prob=0.35):
    """绘制带数字的物理圆盘"""
    cx, cy = center
    
    # 生成随机偏暗的墨黑色彩，提升泛化度
    base_bgr = [12, 12, 12]
    jitter_range = 10
    draw_color = (
        int(np.clip(base_bgr[0] + random.randint(-jitter_range, jitter_range), 0, 45)),
        int(np.clip(base_bgr[1] + random.randint(-jitter_range, jitter_range), 0, 45)),
        int(np.clip(base_bgr[2] + random.randint(-jitter_range, jitter_range), 0, 45))
    )

    # 1. 绘制 6 个轨道
    for idx, (r_in, r_out) in enumerate(track_radii):
        r_in_s = r_in * scale
        r_out_s = r_out * scale
        r_mid_s = (r_in_s + r_out_s) / 2.0
        t_width_px = max(1, int(round((r_out_s - r_in_s))))

        if tracks_config[idx]["type"] == "solid":
            cv2.circle(img, center, int(round(r_mid_s)), draw_color, t_width_px, lineType=cv2.LINE_AA)
        else:
            # 齿状轨道
            w_line_s = w_line * scale
            w_line_px = max(1, int(round(w_line_s)))
            r_outer_rim = r_out_s - w_line_s / 2.0
            cv2.circle(img, center, int(round(r_outer_rim)), draw_color, w_line_px, lineType=cv2.LINE_AA)

            for k in range(N_TEETH):
                center_angle = 90.0 + k * period_deg
                theta1 = center_angle - tooth_deg / 2.0
                theta2 = center_angle + tooth_deg / 2.0
                r_mid_px = int(round(r_mid_s))
                
                cv2.ellipse(img, center, (r_mid_px, r_mid_px), 0, theta1, theta2, draw_color, t_width_px, lineType=cv2.LINE_AA)

    # 2. 绘制完全中心对齐的数字（移除了产生偏差的随机偏移）
    fontsize = int(23 * scale)
    rot_deg = random.uniform(-60, 60)
    text_center = (int(cx), int(cy))

    img = draw_rotated_text_opencv(img, str(target_number), text_center, rot_deg, fontsize, draw_color)

    # 3. 玻璃/塑料壳微弱高光反光
    if not ideal_mode and (random.random() < highlight_prob):
        hl_mask = np.zeros_like(img, dtype=np.uint8)
        hl_angle = random.uniform(-45, 45)
        hl_width = max(1, int(4 * scale))
        hl_height = max(5, int(45 * scale))
        hl_center = (
            cx + int(random.uniform(-12, 12) * scale),
            cy + int(random.uniform(-12, 12) * scale)
        )
        cv2.ellipse(hl_mask, hl_center, (hl_width, hl_height), hl_angle, 0, 360, (255, 255, 255), -1)

        hl_blur_k = int(random.choice([11, 15, 19]) * scale) | 1
        hl_blur_k = max(3, hl_blur_k)
        hl_blur = cv2.GaussianBlur(hl_mask, (hl_blur_k, hl_blur_k), 0)

        gray_hl = cv2.cvtColor(hl_blur, cv2.COLOR_BGR2GRAY)
        alpha_hl = (gray_hl / 255.0) * random.uniform(0.12, 0.32)

        for c in range(3):
            img[:, :, c] = np.clip(img[:, :, c] * (1.0 - alpha_hl) + 255 * alpha_hl, 0, 255).astype(np.uint8)

    return img


# ==================== 数据集标注及流程导出 ====================

def write_yolo_yaml_relative(output_dir):
    """导出 YOLO 配置用的相对路径 dataset.yaml"""
    yaml_path = os.path.join(output_dir, "dataset.yaml")
    names_str = "\n".join([f"  {k}: {v}" for k, v in CLASS_MAP.items()])
    
    yaml_content = f"""# YOLOv5/v8 Dataset Configuration
path: .  # 相对路径
train: images/train
val: images/val

nc: {len(CLASS_MAP)}
names:
{names_str}
"""
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(yaml_content)
    print(f"配置文件已写出: '{yaml_path}'")


def generate_dataset(num_images, output_dir, train_ratio=0.7):
    """主逻辑入口"""
    subdirs = [
        os.path.join("images", "train"),
        os.path.join("images", "val"),
        os.path.join("labels", "train"),
        os.path.join("labels", "val")
    ]
    for subdir in subdirs:
        os.makedirs(os.path.join(output_dir, subdir), exist_ok=True)

    print(f"开始生成同心圆带数字数据集，目标: {num_images} 张...")
    width, height = IMAGE_SIZE

    train_count = int(num_images * train_ratio)
    val_count = num_images - train_count
    splits = ["train"] * train_count + ["val"] * val_count
    random.shuffle(splits)

    generated_count = 0

    while generated_count < num_images:
        is_ideal_mode = (random.random() < IDEAL_MODE_RATIO)
        
        # 1. 灰白渐变加暗影背景
        img = generate_white_harsh_background(width, height, ideal_mode=is_ideal_mode, limit_shadows=True)

        # 2. 地面底层干涉
        forbidden_bgrs = [(10, 10, 10), (30, 30, 30)]
        if not is_ideal_mode:
            draw_geometric_interference(img, width, height, forbidden_bgrs=forbidden_bgrs, draw_on_top=False)

        # 3. 碰撞检测与多目标放置（单张图随机 1 ~ 3 个目标）
        placed_objects = []
        num_targets = random.randint(1, 3)
        selected_numbers = [random.choice([1, 2, 3]) for _ in range(num_targets)]

        for target_number in selected_numbers:
            scale = random.uniform(MIN_SCALE, MAX_SCALE)
            R_base = int(r_out_6 * scale) # 外层圆环半径约 49.6

            attempts = 0
            placed = False

            while attempts < 200:
                margin = int(R_base * 1.15)
                cx = random.randint(margin, width - margin)
                cy = random.randint(margin, height - margin)

                collision = False
                for prev_cx, prev_cy, prev_scale, _ in placed_objects:
                    prev_R_base = int(r_out_6 * prev_scale)
                    distance = np.sqrt((cx - prev_cx) ** 2 + (cy - prev_cy) ** 2)
                    if distance < (R_base + prev_R_base) * 1.15:
                        collision = True
                        break

                if not collision:
                    placed_objects.append((cx, cy, scale, target_number))
                    placed = True
                    break

                attempts += 1

        if len(placed_objects) == 0:
            continue

        placed_objects.sort(key=lambda o: o[1]) # 按 Y 轴物理遮挡排序

        # 4. 渲染同心圆圆环盘（数字已经过高精度居中修正）
        for cx, cy, scale, target_number in placed_objects:
            img = draw_encoder_pattern(img, (cx, cy), target_number, scale, is_ideal_mode, HIGHLIGHT_PROB)

        # 5. 表层覆盖物理干涉
        if not is_ideal_mode:
            draw_geometric_interference(img, width, height, forbidden_bgrs=forbidden_bgrs, draw_on_top=True)

        # 6. 镜头边缘失焦模糊
        if not is_ideal_mode and (random.random() < REGIONAL_BLUR_PROB):
            img = apply_regional_blur(img, BLUR_GRID_DIV, MIN_BLUR_KERNEL, MAX_BLUR_KERNEL)

        # 7. 传感器噪点叠加
        noise_level = 0.0 if is_ideal_mode else random.uniform(0.1, 0.85)
        img = apply_camera_noise_dynamic(img, noise_level)

        current_split = splits[generated_count]

        # 8. 构造唯一且直观的文件名称
        filename_parts = []
        for cx, cy, _, target_number in placed_objects:
            filename_parts.append(f"{target_number}({cx}, {cy})")
        ideal_flag = "ideal_" if is_ideal_mode else f"noise_{noise_level:.2f}_"
        base_name = f"{ideal_flag}" + "".join(filename_parts)

        img_filename = f"{base_name}.jpg"
        label_filename = f"{base_name}.txt"

        img_save_path = os.path.join(output_dir, "images", current_split, img_filename)
        label_save_path = os.path.join(output_dir, "labels", current_split, label_filename)

        # 存储大图
        save_image_robust(img, img_save_path)

        # 9. 导出 YOLO 标准标注文本
        with open(label_save_path, "w", encoding="utf-8") as f_lbl:
            for cx, cy, scale, target_number in placed_objects:
                # 类别映射：class_0->'1', class_1->'2', class_2->'3'
                class_id = target_number - 1
                R_base = int(r_out_6 * scale)
                
                # 严格边界约束
                xmin = max(0, cx - R_base)
                ymin = max(0, cy - R_base)
                xmax = min(width, cx + R_base)
                ymax = min(height, cy + R_base)

                x_center = (xmin + xmax) / 2.0 / width
                y_center = (ymin + ymax) / 2.0 / height
                bbox_w = (xmax - xmin) / width
                bbox_h = (ymax - ymin) / height

                f_lbl.write(f"{class_id} {x_center:.6f} {y_center:.6f} {bbox_w:.6f} {bbox_h:.6f}\n")

        generated_count += 1
        if generated_count % 50 == 0 or generated_count == num_images:
            print(f"进度反馈: {generated_count}/{num_images} 张图像构建完毕并输出存盘。")

    write_yolo_yaml_relative(output_dir)
    print(f"\n数据集成功生成并归档。本地保存路径: '{output_dir}'")


if __name__ == "__main__":
    generate_dataset(NUM_IMAGES, SAVE_DIR, TRAIN_RATIO)