"""画一对沙盘图的 LightGlue 匹配连线图：绿色=几何验证内点，红色=外点。"""
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from lightglue import LightGlue, SuperPoint
from lightglue.utils import load_image, rbd

device = 'cpu'
base = '/home/e0h/nexus_workspace/NEXUS/data/processed/nexus_sandbox_gdr_net_v01/train_pbr/000001/rgb'
p0 = f'{base}/000001.png'
p1 = f'{base}/000003.png'

extractor = SuperPoint(max_num_keypoints=2048).eval().to(device)
matcher = LightGlue(features='superpoint').eval().to(device)
im0 = load_image(p0).to(device)
im1 = load_image(p1).to(device)
f0 = extractor.extract(im0)
f1 = extractor.extract(im1)
m = matcher({'image0': f0, 'image1': f1})
f0, f1, m = [rbd(x) for x in [f0, f1, m]]
matches = m['matches']
k0 = f0['keypoints'][matches[..., 0]].cpu().numpy()
k1 = f1['keypoints'][matches[..., 1]].cpu().numpy()

_, mask = cv2.findFundamentalMat(k0, k1, cv2.FM_RANSAC, 3.0, 0.999)
inl = mask.ravel().astype(bool)

img0 = cv2.imread(p0)
img1 = cv2.imread(p1)
H0, W0 = img0.shape[:2]
H1, W1 = img1.shape[:2]
canvas = np.ones((max(H0, H1), W0 + W1, 3), dtype=np.uint8) * 255
canvas[:H0, :W0] = img0
canvas[:H1, W0:] = img1

fig, ax = plt.subplots(figsize=(16, 9))
ax.imshow(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
for i in range(len(k0)):
    color = 'lime' if inl[i] else 'red'
    ax.plot([k0[i, 0], k1[i, 0] + W0], [k0[i, 1], k1[i, 1]], color=color, linewidth=0.5, alpha=0.6)
ax.scatter(k0[:, 0], k0[:, 1], s=8, c='cyan', zorder=3)
ax.scatter(k1[:, 0] + W0, k1[:, 1], s=8, c='cyan', zorder=3)
ax.axis('off')
ax.set_title(f'LightGlue: {len(k0)} matches, {int(inl.sum())} inliers (green)', fontsize=14)
out = '/home/e0h/nexus_workspace/NEXUS/experiments/runs/2026-08-30_E007_lightglue_sandbox/match_000001_000003.png'
plt.savefig(out, bbox_inches='tight', dpi=120)
print('saved:', out)
