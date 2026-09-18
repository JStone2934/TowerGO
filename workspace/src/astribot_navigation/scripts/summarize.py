#!/usr/bin/env python3
import json,argparse
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('reports');a=p.parse_args();root=Path(a.reports)
reports=[]
for folder in sorted(root.glob('acceptance-*')):
 if folder.is_dir():
  for file in sorted(folder.glob('*/report.json')):
   row=json.loads(file.read_text());row['path']=str(file.relative_to(root));reports.append(row)
mapping=[r for r in reports if r['mode']=='mapping'];navigation=[r for r in reports if r['mode']=='navigation'];faults=[r for r in reports if r['mode']=='faults']
goals=[g for r in navigation for g in r.get('goals',[])]
summary={'mapping_runs':len(mapping),'mapping_passed':sum(r['passed'] for r in mapping),'navigation_runs':len(navigation),'goals':len(goals),'goals_passed':sum(g['passed'] for g in goals),'fault_runs':len(faults),'faults_passed':sum(r['passed'] for r in faults),'failed_runs':[r['path'] for r in reports if not r['passed']]}
summary['complete']=len(mapping)==12 and len(navigation)==9 and len(goals)==27 and len(faults)==1
summary['passed']=summary['complete'] and not summary['failed_runs']
(root/'acceptance-summary.json').write_text(json.dumps(summary,indent=2))
lines=['# TowerGO 实现与验证状态','',f"完整矩阵：{'通过' if summary['passed'] else '未全部通过或尚未完成'}",'',
 f"- 建图：{summary['mapping_passed']}/{len(mapping)} 个实验通过（预期12）。",
 f"- 导航：{summary['goals_passed']}/{len(goals)} 个目标通过（预期27）。",
 f"- 异常实验：{summary['faults_passed']}/{len(faults)} 组通过（预期1）。"]
if mapping:
 good=[r for r in mapping if 'position_rmse_m' in r]
 if good:lines += [f"- 建图位置 RMSE 范围：{min(r['position_rmse_m'] for r in good):.4f}–{max(r['position_rmse_m'] for r in good):.4f} m。",f"- 建图墙面误差 P95 最大值：{max(r['wall_error_p95_m'] for r in good):.4f} m。"]
if goals:lines += [f"- 导航终点位置误差最大值：{max(g['position_error_m'] for g in goals):.4f} m。",f"- 导航终点角度误差最大值：{max(g['yaw_error_deg'] for g in goals):.2f}°。"]
audit=root/'map-coverage-audit.json'
if audit.exists():
 rows=json.loads(audit.read_text())
 if rows:lines += [f"- 双向地图审计：墙面覆盖率最低 {min(x['true_wall_coverage_within_15cm'] for x in rows):.2%}；错误自由空间比例最大 {max(x['unexplained_free_fraction'] for x in rows):.2%}。"]
unit=root/'unit-container.log'
if unit.exists():lines+=['','## 单元与集成测试','',unit.read_text().strip()]
replay=root/'replay-check/report.json'
if replay.exists():lines+=['',f"录包暂停、恢复、完整重启验证：{'通过' if json.loads(replay.read_text())['passed'] else '失败'}。"]
lines+=['','SDK 状态历史录包：8759 条消息解析完成，坐标语义未确认，未发布 TF。','',
'详细结果见 acceptance-summary.json、各实验 report.json、map-coverage-audit.json 和 launch.log。早期调试失败记录保留，不计入最终验收矩阵。','',
'全部运行均为隔离容器中的合成实验，不导入 SDK，不控制真实机器人。模拟精度不能代表实机精度。']
if summary['failed_runs']:lines+=['','未通过的实验：']+['- '+x for x in summary['failed_runs']]
(root/'IMPLEMENTATION_STATUS.md').write_text('\n'.join(lines)+'\n');print(json.dumps(summary,indent=2))
