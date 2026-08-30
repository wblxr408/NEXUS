"""LightGlue 沙盘特征匹配复现 —— 按组长「13 项指标模板」生成完整报告。

指标来源：docs/算法比较定义参数.md（组长给定的统一评估框架）。

说明：
- 前 6 项（3D Position RMSE/MAE、P50/P95、Maximum Error、Axis Bias）是位置误差，
  LightGlue 是特征匹配器、不输出位置，标记为 N/A，由 PnP/PVNet 位姿环节填写。
- Availability/Success Rate 定义：该图对「内点数 >= 8」（够 RANSAC 基础矩阵 / 最小 PnP）视为成功。
- Failure Rate = 1 - Availability。
- Latency 在离线两两匹配下等于 Runtime（无传感器采样环节）。
- Recovery Time 离线无失锁概念，N/A。
"""
import argparse
import glob
import os
import resource
import time

import cv2
import numpy as np
import torch

from lightglue import LightGlue, SuperPoint
from lightglue.utils import load_image, rbd

MIN_INLIERS = 8  # 内点少于该值视为该图对匹配失败


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
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    extractor = SuperPoint(max_num_keypoints=args.max_keypoints).eval().to(device)
    matcher = LightGlue(features='superpoint').eval().to(device)

    os.makedirs(args.out_dir, exist_ok=True)

    rows = []
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            n_m, n_in, ratio, rt_ms = match_pair(extractor, matcher, paths[i], paths[j], device)
            rows.append((i, j, n_m, n_in, ratio, rt_ms))

    n_m_arr = np.array([r[2] for r in rows], dtype=float)
    n_in_arr = np.array([r[3] for r in rows], dtype=float)
    ratio_arr = np.array([r[4] for r in rows], dtype=float)
    rt_arr = np.array([r[5] for r in rows], dtype=float)

    peak_mem_mb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
    rt_mean = float(rt_arr.mean())
    rt_p95 = float(np.percentile(rt_arr, 95))
    update_rate_hz = 1000.0 / rt_mean if rt_mean > 0 else 0.0

    success = n_in_arr >= MIN_INLIERS
    n_success = int(success.sum())
    availability = n_success / len(rows)
    failure_rate = 1.0 - availability

    # CSV
    csv_path = os.path.join(args.out_dir, 'matching_metrics.csv')
    with open(csv_path, 'w') as f:
        f.write('image_i,image_j,n_matches,n_inliers,inlier_ratio,runtime_ms,success\n')
        for (i, j, n_m, n_in, ratio, rt_ms), ok in zip(rows, success):
            f.write(f'{i},{j},{n_m},{n_in},{ratio:.4f},{rt_ms:.2f},{int(ok)}\n')

    # Markdown 报告（组长模板）
    md_path = os.path.join(args.out_dir, 'lightglue_report.md')
    with open(md_path, 'w', encoding='utf-8') as f:
        f.write('# LightGlue 沙盘特征匹配复现报告\n\n')
        f.write(f'- 数据：{len(paths)} 张沙盘 RGB 图，两两配对共 {len(rows)} 对\n')
        f.write(f'- 特征提取：SuperPoint（max_keypoints={args.max_keypoints}）\n')
        f.write('- 匹配器：LightGlue（features=superpoint）\n')
        f.write(f'- 运行设备：{device}\n')
        f.write('- 几何验证：RANSAC 基础矩阵（阈值 3.0px，置信度 0.999）\n\n')

        f.write('## 位置误差定义（组长给定）\n\n')
        f.write('```text\n')
        f.write('e_i = p_bar_i - p_i^gt\n')
        f.write('d_i = ||e_i||_2\n')
        f.write('RMSE = sqrt( (1/N) * sum_i d_i^2 )\n')
        f.write('```\n\n')
        f.write('> 位置误差类指标针对「输出三维位置的算法」（PnP、PVNet、融合）。\n')
        f.write('> LightGlue 是特征匹配器，只输出点对应关系、不输出位置，故此类指标标记 N/A，由位姿环节填写。\n\n')

        f.write('## 核心指标（组长 13 项模板）\n\n')
        f.write('| 指标 | LightGlue 数值 | 适用性 |\n')
        f.write('|---|---|---|\n')
        f.write('| 3D Position RMSE | N/A | 位置类，属 PnP/PVNet |\n')
        f.write('| 3D Position MAE | N/A | 位置类，属 PnP/PVNet |\n')
        f.write('| P50 / Median Error | N/A | 位置类，属 PnP/PVNet |\n')
        f.write('| P95 Error | N/A | 位置类，属 PnP/PVNet |\n')
        f.write('| Maximum Error | N/A | 位置类，属 PnP/PVNet |\n')
        f.write('| Axis Bias | N/A | 位置类，属 PnP/PVNet |\n')
        f.write(f'| Availability / Success Rate | {availability:.1%}（{n_success}/{len(rows)} 对内点≥{MIN_INLIERS}） | ✓ |\n')
        f.write(f'| Failure Rate | {failure_rate:.1%}（{len(rows) - n_success}/{len(rows)}） | ✓ |\n')
        f.write(f'| Update Rate | {update_rate_hz:.3f} Hz | ✓ |\n')
        f.write(f'| Latency（mean / P95） | {rt_mean:.1f} / {rt_p95:.1f} ms | ✓（离线=解算耗时） |\n')
        f.write('| Recovery Time | N/A | 离线两两匹配，无失锁概念 |\n')
        f.write(f'| Runtime | {rt_mean:.1f} ms | ✓ |\n')
        f.write(f'| CPU/GPU/Memory | {device} / 峰值 {peak_mem_mb:.0f} MB | ✓ |\n\n')

        f.write('## 特征匹配专属诊断指标（文档第四部分）\n\n')
        f.write('| 指标 | 数值 |\n')
        f.write('|---|---|\n')
        f.write(f'| Inlier Count（均值） | {n_in_arr.mean():.1f} |\n')
        f.write(f'| Matching Inlier Ratio（均值） | {ratio_arr.mean():.3f} |\n\n')

        f.write('## 两两匹配明细\n\n')
        f.write('| 图i | 图j | 匹配数 | 内点数 | 内点比 | 耗时(ms) | 成功 |\n')
        f.write('|---|---|---|---|---|---|---|\n')
        for (i, j, n_m, n_in, ratio, rt_ms), ok in zip(rows, success):
            f.write(f'| {i} | {j} | {n_m} | {n_in} | {ratio:.3f} | {rt_ms:.1f} | {"✓" if ok else "✗"} |\n')

    print('=== 汇总 ===')
    print(f'Inlier Count 均值: {n_in_arr.mean():.1f}')
    print(f'Matching Inlier Ratio 均值: {ratio_arr.mean():.3f}')
    print(f'Runtime: {rt_mean:.1f}ms (mean) / {rt_p95:.1f}ms (P95)')
    print(f'Update Rate: {update_rate_hz:.3f} Hz')
    print(f'Availability: {availability:.1%} ({n_success}/{len(rows)})')
    print(f'Failure Rate: {failure_rate:.1%}')
    print(f'Peak Memory: {peak_mem_mb:.0f} MB, 设备: {device}')
    print(f'CSV: {csv_path}')
    print(f'报告: {md_path}')


if __name__ == '__main__':
    main()
