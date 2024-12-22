import os
from glob import glob
import re
from PIL import Image

def create_gif_from_images(image_folder, output_path, duration=0.2):
    """
    将文件夹中的图片转换为GIF动画
    
    参数:
    image_folder: 图片所在文件夹路径
    output_path: 输出GIF的路径
    duration: 每帧持续时间（秒）
    """
    # 获取所有png图片
    images = glob(os.path.join(image_folder, "*.png"))
    
    # 使用正则表达式提取数字
    def extract_number(filename):
        numbers = re.findall(r'\d+', filename)
        return int(numbers[-1]) if numbers else 0
    
    # 按数字排序
    images.sort(key=extract_number)
    
    if not images:
        print("No images found in the specified folder!")
        return
    
    # 读取所有图片
    frames = []
    total_images = len(images)
    
    for i, image_path in enumerate(images, 1):
        frame = Image.open(image_path)
        frames.append(frame)
        print(f"Processing image {i}/{total_images}: {os.path.basename(image_path)}")
    
    # 保存为GIF
    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=int(duration * 1000),  # 转换为毫秒
        loop=0
    )
    
    print(f"\nGIF created successfully: {output_path}")

if __name__ == "__main__":
    # 设置输入输出路径
    image_folder = r"C:\Users\11967\cursor_files\simulation_of_robotic_map\output\auto_drive\reconstruction"
    output_path = r"C:\Users\11967\cursor_files\simulation_of_robotic_map\output\auto_drive.gif"
    
    # 创建GIF
    create_gif_from_images(image_folder, output_path, duration=0.25) 