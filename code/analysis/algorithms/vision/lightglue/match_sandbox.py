"""在沙盘 RGB 图上跑 LightGlue 特征匹配，输出 Inlier Count / Matching Inlier Ratio / Runtime。

输入：generate_gdr_net_dataset.py 生成的沙盘 RGB 图（多视角、有重叠）
指标定义见 docs/算法比较定义参数.md 第四部分「视觉算法还应报告的诊断指标」。
"""
import argparse
import glob
import os
import time

import cv2
import numpy as np
import torch

from lightglue import LightGlue, SuperPoint
from lightglue.utils import load_image, rbd


def match_pair(extractor, matcher, path0, path1, device='cpu'):
    """匹配两张图，返回 (n_matches, n_inliers, inlier_ratio, runtime_ms)。"""
    image0 = load_image(path0).to(device)
    image1 = load_image(path1).to(device)
    t0 = time.time()
    feats0 = extractor.extract(image0)
    feats1 = extractor.extract(image1)
    m = matcher({'image0': feats0, 'image1': feats1})
    feats0, feats1, m = [rbd(x) for x in [feats0, feats1, m]]
    runtime_ms = (time.time() - t0) * 1000.0

    matches = m['matches']
    n_matches = int(matches.shape[0])
    k0 = feats0['keypoints'][matches[..., 0]].cpu().numpy()
    k1 = feats1['keypoints'][matches[..., 1]].cpu().numpy()

    # 用 RANSAC 基础矩阵做几何验证，统计内点（通过验证 = 内点）
    if n_matches >= 8:
        _, mask = cv2.findFundamentalMat(k0, k1, cv2.FM_RANSAC, 3.0, 0.999)
        n_inliers = int(mask.sum()) if mask is not None else 0
    else:
        n_inliers = 0
    ratio = (n_inliers / n_matches) if n_matches else 0.0
    return n_matches, n_inliers, ratio, runtime_ms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rgb_dir',
                    default='/home/e0h/nexus_workspace/NEXUS/data/processed/nexus_sandbox_gdr_net_v01/train_pbr/000001/rgb')
    ap.add_argument('--out_dir',
                    default='/home/e0h/nexus_workspace/NEXUS/experiments/runs/2026-08-30_E007_lightglue_sandbox')
    ap.add_argument('--max_keypoints', type=int, default=2048)
    args = ap.parse_args()

    paths = sorted(glob.glob(os.path.join(args.rgb_dir, '*.png')))
    print(f'共 {len(paths)} 张沙盘图: {[os.path.basename(p) for p in paths]}')

    device = 'cpu'
    extractor = SuperPoint(max_num_keypoints=args.max_keypoints).eval().to(device)
    matcher = LightGlue(features='superpoint').eval().to(device)

    os.makedirs(args.out_dir, exist_ok=True)

    rows = []
    print('\n=== 两两匹配结果 ===')
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            n_m, n_in, ratio, rt_ms = match_pair(extractor, matcher, paths[i], paths[j], device)
            rows.append((i, j, n_m, n_in, ratio, rt_ms))
            print(f'图{i} vs 图{j}: 匹配 {n_m} 对, 内点 {n_in}, 内点比 {ratio:.3f}, 耗时 {rt_ms:.1f}ms')

    n_in_arr = np.array([r[3] for r in rows], dtype=float)
    ratio_arr = np.array([r[4] for r in rows], dtype=float)
    rt_arr = np.array([r[5] for r in rows], dtype=float)
    print('\n=== 汇总（均值）===')
    print(f'Inlier Count 均值: {n_in_arr.mean():.1f}')
    print(f'Matching Inlier Ratio 均值: {ratio_arr.mean():.3f}')
    print(f'Runtime 均值: {rt_arr.mean():.1f}ms')

    csv_path = os.path.join(args.out_dir, 'matching_metrics.csv')
    with open(csv_path, 'w') as f:
        f.write('image_i,image_j,n_matches,n_inliers,inlier_ratio,runtime_ms\n')
        for i, j, n_m, n_in, ratio, rt_ms in rows:
            f.write(f'{i},{j},{n_m},{n_in},{ratio:.4f},{rt_ms:.2f}\n')
    print(f'\n汇总已保存: {csv_path}')


if __name__ == '__main__':
    main()
