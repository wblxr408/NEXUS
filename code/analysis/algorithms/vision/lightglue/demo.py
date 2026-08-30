import torch
import numpy as np
import matplotlib
matplotlib.use('Agg')  # 无界面环境用 Agg 后端，只保存图片不弹窗
import matplotlib.pyplot as plt
from lightglue import LightGlue, SuperPoint
from lightglue.utils import load_image, rbd

device = 'cpu'  # 你没有 GPU，用 CPU 跑

print("加载图片...")
image0 = load_image('assets/DSC_0410.JPG').to(device)
image1 = load_image('assets/DSC_0411.JPG').to(device)

print("加载模型（第一次会自动下载权重，耐心等）...")
extractor = SuperPoint(max_num_keypoints=2048).eval().to(device)
matcher = LightGlue(features='superpoint').eval().to(device)

print("提取特征并匹配...")
feats0 = extractor.extract(image0)
feats1 = extractor.extract(image1)
matches01 = matcher({'image0': feats0, 'image1': feats1})
feats0, feats1, matches01 = [rbd(x) for x in [feats0, feats1, matches01]]

matches = matches01['matches']
points0 = feats0['keypoints'][matches[..., 0]]
points1 = feats1['keypoints'][matches[..., 1]]
print(f"匹配到 {matches.shape[0]} 对特征点")

# 画图：两张图左右拼一起，连线表示匹配关系
k0 = points0.cpu().numpy()
k1 = points1.cpu().numpy()
im0 = (image0.permute(1, 2, 0).cpu().numpy() * 255).astype(np.uint8)
im1 = (image1.permute(1, 2, 0).cpu().numpy() * 255).astype(np.uint8)
H0, W0 = im0.shape[:2]
H1, W1 = im1.shape[:2]
H = max(H0, H1)
canvas = np.ones((H, W0 + W1, 3), dtype=np.uint8) * 255
canvas[:H0, :W0] = im0
canvas[:H1, W0:] = im1

fig, ax = plt.subplots(figsize=(16, 9))
ax.imshow(canvas)
for i in range(len(k0)):
    x0, y0 = k0[i]
    x1, y1 = k1[i]
    ax.plot([x0, x1 + W0], [y0, y1], color='lime', linewidth=0.6, alpha=0.6)
ax.scatter(k0[:, 0], k0[:, 1], s=10, c='red', marker='o', zorder=3)
ax.scatter(k1[:, 0] + W0, k1[:, 1], s=10, c='red', marker='o', zorder=3)
ax.axis('off')
plt.savefig('match_result.png', bbox_inches='tight', dpi=120)
print("结果图已保存到 match_result.png")
