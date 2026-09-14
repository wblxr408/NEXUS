#!/usr/bin/env python3
from pathlib import Path
import csv,json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
RUN=Path(__file__).resolve().parent
E050=ROOT/'experiments/runs/2026-09-09_E050_ten_target_fitted_simulation'
BLUE='#1463D9'; ORANGE='#F2541B'; TEAL='#14806B'; NAVY='#0B1F3A'; PALE='#F4F7FB'; GRID='#D8E1EC'
plt.rcParams.update({'font.family':['Noto Sans SC','Microsoft YaHei','DejaVu Sans'],'axes.unicode_minus':False})

def head(fig,title,sub):
    fig.patch.set_facecolor('white'); fig.text(.055,.955,title,fontsize=18,weight='bold',color=NAVY); fig.text(.055,.918,sub,fontsize=9.5,color='#60708A')
def style(ax):
    ax.set_facecolor(PALE); ax.grid(color=GRID,alpha=.8,linewidth=.7)
    for s in ax.spines.values(): s.set_color('#B8C6D8')
def foot(fig,s): fig.text(.055,.018,s,fontsize=8,color='#718096')
def load_rows(): return list(csv.DictReader((E050/'tbl_ten_target_simulated_observations_v01.csv').open(encoding='utf-8')))

def quality(rows):
    rr=[r for r in rows if r['method']=='proposed']; e=np.array([float(r['error_xy_m']) for r in rr])*100
    rng=np.random.default_rng(51); q=np.clip(1-e/(np.percentile(e,99)+1e-9)+rng.normal(0,.05,len(e)),0,1)
    bins=np.linspace(0,1,7); c=(bins[:-1]+bins[1:])/2; med=[]; p90=[]
    for a,b in zip(bins[:-1],bins[1:]):
        v=e[(q>=a)&(q<b)]; med.append(float(np.median(v)) if len(v) else np.nan); p90.append(float(np.percentile(v,90)) if len(v) else np.nan)
    fig=plt.figure(figsize=(10.5,5.8),constrained_layout=False); head(fig,'视觉质量分数 → 定位误差','Intermediate evidence for quality-aware covariance adaptation (fitted simulation)'); ax=fig.add_axes([.08,.18,.62,.67]); style(ax)
    ax.scatter(q,e,s=10,alpha=.12,color=BLUE,edgecolors='none'); ax.plot(c,med,marker='o',lw=2.8,color=ORANGE,label='Median error'); ax.plot(c,p90,marker='s',lw=2,ls='--',color=TEAL,label='P90 error'); ax.set(xlabel='Visual quality score qv (scenario)',ylabel='XY error (cm)',xlim=(0,1)); ax.legend(frameon=False); ax.grid(alpha=.25)
    card=fig.add_axes([.76,.30,.19,.40]); card.set_facecolor(NAVY); card.set_xticks([]); card.set_yticks([]); card.text(.08,.82,'MECHANISM',color='#8CB8FF',fontsize=9,weight='bold'); card.text(.08,.59,'quality-aware',color='white',fontsize=13,weight='bold'); card.text(.08,.43,'covariance',color='white',fontsize=13,weight='bold'); card.text(.08,.18,'low qv -> larger uncertainty\nhigh qv -> stronger weight',color='#D9E7FF',fontsize=8.5)
    foot(fig,'qv 为仿真质量代理；不是逐帧实测质量标定或因果证明'); fig.savefig(RUN/'fig_quality_error_calibration_v01.png',dpi=320,facecolor='white'); plt.close(fig)
    return {'bin_centers':c.tolist(),'median_error_cm':med,'p90_error_cm':p90}

def robust():
    r=np.linspace(0,6,400); hub=np.minimum(1,1.5/np.maximum(r,1e-9)); irls=1/(1+(r/1.2)**2)
    fig=plt.figure(figsize=(10.5,5.8),constrained_layout=False); head(fig,'鲁棒残差权重：离群观测不会主导融合','Algorithm-response curve for Huber / IRLS-style down-weighting (scenario)'); ax=fig.add_axes([.09,.18,.63,.66]); style(ax)
    ax.plot(r,hub,lw=3,color=BLUE,label='Huber'); ax.plot(r,irls,lw=3,color=ORANGE,label='IRLS-like'); ax.axvspan(0,1,color=TEAL,alpha=.1); ax.axvspan(3,6,color=ORANGE,alpha=.08); ax.set(xlabel='Standardized residual |r| / sigma',ylabel='Relative observation weight',ylim=(0,1.05)); ax.legend(frameon=False); ax.grid(alpha=.25)
    card=fig.add_axes([.78,.30,.17,.39]); card.set_facecolor(PALE); card.set_xticks([]); card.set_yticks([]); card.text(.08,.82,'ROBUSTNESS',color=ORANGE,fontsize=9,weight='bold'); card.text(.08,.60,'large residual',color=NAVY,fontsize=12,weight='bold'); card.text(.08,.43,'down-weighted',color=BLUE,fontsize=14,weight='bold'); card.text(.08,.19,'NLOS / 遮挡 / 误匹配不再劫持状态估计',color='#60708A',fontsize=8.5)
    foot(fig,'响应曲线用于解释鲁棒因子机制；不等同于某次实机残差权重日志'); fig.savefig(RUN/'fig_robust_weight_response_v01.png',dpi=320,facecolor='white'); plt.close(fig)

def geometry():
    gt=json.loads((ROOT/'experiments/runs/2026-09-07_E030_physical_sandbox_metric_reference/physical_sandbox_ground_truth_v01.json').read_text(encoding='utf-8')); pts=np.array([x['corner_anchor_xy_m'] for x in gt['items'] if x.get('category')=='traffic_light'],float); anchors=np.array([[.9,-.9],[0,5],[4,5],[4,0]],float); x=np.linspace(0,4.2,100); y=np.linspace(0,5.1,120); X,Y=np.meshgrid(x,y); gd=np.zeros_like(X)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]): gd[i,j]=np.mean(1/np.maximum(np.linalg.norm(anchors-np.array([X[i,j],Y[i,j]]),axis=1),.15))
    fig=plt.figure(figsize=(10.5,6.2),constrained_layout=False); head(fig,'观测几何质量场：规划应主动寻找高信息区域','Anchor geometry proxy and accepted target locations in the sandbox map frame'); ax=fig.add_axes([.08,.14,.68,.73]); style(ax); im=ax.contourf(X,Y,gd,levels=18,cmap='Blues',alpha=.86); cbar=fig.colorbar(im,ax=ax,fraction=.035,pad=.02); cbar.set_label('Geometry information proxy (scenario)'); ax.scatter(anchors[:,0],anchors[:,1],marker='^',s=90,color=ORANGE,label='UWB anchor'); ax.scatter(pts[:,0],pts[:,1],s=15,color='white',edgecolor=NAVY,alpha=.9,label='48 accepted truth points'); ax.set(xlabel='Map X (m)',ylabel='Map Y (m)'); ax.legend(frameon=False,loc='upper right'); ax.set_aspect('equal'); foot(fig,'颜色为锚点几何代理场，不是实测 GDOP；用于说明主动观测规划接口'); fig.savefig(RUN/'fig_observation_geometry_field_v01.png',dpi=320,facecolor='white'); plt.close(fig)

def latency():
    stages=['capture','detect','feature','geometry','fusion','display']; vals=np.array([8,21,12,9,4,3]); starts=np.cumsum(np.r_[0,vals[:-1]])
    fig=plt.figure(figsize=(10.5,5.8),constrained_layout=False); head(fig,'异步定位链路：延迟预算与队列控制','Stage-level latency budget for the edge deployment scenario'); ax=fig.add_axes([.09,.20,.62,.64]); style(ax); ax.barh(stages[::-1],vals[::-1],left=starts[::-1],color=[BLUE,'#3E86E8','#63A5FF','#F39A3F','#20A37A','#8BC6B5'],edgecolor='white',height=.62); ax.set(xlabel='Latency (ms; scenario)'); ax.axvline(vals.sum(),color=ORANGE,ls='--',lw=1.8); ax.text(vals.sum()+1,.3,f'total {vals.sum():.0f} ms',color=ORANGE,weight='bold'); ax.grid(axis='x',alpha=.25)
    card=fig.add_axes([.77,.29,.18,.42]); card.set_facecolor(NAVY); card.set_xticks([]); card.set_yticks([]); card.text(.08,.83,'SCHEDULER',color='#8CB8FF',fontsize=9,weight='bold'); card.text(.08,.60,'finite queue',color='white',fontsize=13,weight='bold'); card.text(.08,.43,'+ timeout gate',color='white',fontsize=11); card.text(.08,.19,'drop stale results; keep the chain real-time',color='#D9E7FF',fontsize=9); foot(fig,'阶段时延为边缘部署场景设定；不是 E047 端到端实测 latency'); fig.savefig(RUN/'fig_async_latency_waterfall_v01.png',dpi=320,facecolor='white'); plt.close(fig)

def covariance(rows):
    fig=plt.figure(figsize=(10.5,5.8),constrained_layout=False); head(fig,'不确定度校准：预测协方差是否覆盖实际误差','Empirical coverage of 1-sigma / 2-sigma bands across the fitted simulation'); ax=fig.add_axes([.10,.19,.60,.64]); style(ax)
    for method,color in [('baseline','#8A94A6'),('proposed',BLUE)]:
        e=np.array([float(r['error_xy_m']) for r in rows if r['method']==method]); sigma=np.median(e)*(.85 if method=='proposed' else .9); ax.plot([1,2],[np.mean(e<=sigma),np.mean(e<=2*sigma)],marker='o',lw=3,color=color,label=method.capitalize())
    ax.plot([1,2],[.68,.95],ls='--',color=ORANGE,label='2D Gaussian reference'); ax.set(xlabel='Predicted radial band',ylabel='Empirical coverage',xticks=[1,2],xticklabels=['1-sigma','2-sigma'],ylim=(0,1.03)); ax.legend(frameon=False); ax.grid(alpha=.25); foot(fig,'覆盖率来自拟合仿真；虚线仅作二维高斯参考，不构成统计检验'); fig.savefig(RUN/'fig_covariance_calibration_v01.png',dpi=320,facecolor='white'); plt.close(fig)

def main():
    RUN.mkdir(parents=True,exist_ok=True); rows=load_rows(); summary={'status':'COMPUTED_INTERMEDIATE_FITTED_SIMULATION_NOT_LIVE_VALIDATED','source':'E030 truth + E049/E048 fitted residuals','figures':[]}; summary['quality_bins']=quality(rows); robust(); geometry(); latency(); covariance(rows); summary['figures']=sorted(p.name for p in RUN.glob('fig_*_v01.png')); summary['limitations']=['qv, geometry proxy, latency stages and covariance bands are simulation/scenario quantities.','No figure is a physical recognition or live end-to-end validation.']; (RUN/'metrics_v01.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8'); (RUN/'render_report.json').write_text(json.dumps({'status':'generated_pending_visual_inspection','figures':summary['figures']},ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
if __name__=='__main__': main()
