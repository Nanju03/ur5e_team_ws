"""Export the completed isolated campaign without changing the original PDF."""
import json,html,statistics,os,math,csv
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QTextDocument,QFont
from PyQt5.QtPrintSupport import QPrinter
ROOT=Path(__file__).resolve().parent
rows=json.loads((ROOT/'results.json').read_text())['results']
assert len(rows)==32 and {r['number'] for r in rows}==set(range(1,33)), 'All 32 results are required'
manifest=json.loads((ROOT/'manifest.json').read_text())
resume_path=ROOT/'resume_manifest.json'
resume=json.loads(resume_path.read_text()) if resume_path.exists() else {}
measurement_history=''
if resume.get('resumed_at'):
 measurement_history=(
  '시험 도중 같은 PC에서 실물 작업이 실행 중임을 확인해 가상 시험만 일시 중단했다. '
  f"완료한 {len(resume['completed_case_numbers'])}개 결과를 보존하고, 사용자의 실물 테스트 종료 확인 후 "
  f"남은 {len(resume['remaining_case_numbers'])}개만 재개했다. "
  '중단 전후의 동시 작업 조건이 완전히 같지 않으므로 결과 비교에 이 차이를 고려해야 한다. '
  '실물 작업과의 동시 실행이 개별 실패의 원인이었다고 확정하지 않는다.')
def diagnostic_records(reason):
 decoder=json.JSONDecoder();start=reason.find('{')
 while start>=0:
  try:
   record,_=decoder.raw_decode(reason[start:])
   if isinstance(record,dict):yield record
  except (ValueError,TypeError):pass
  start=reason.find('{',start+1)
def gripper_feedback_mismatch(reason):
 for record in diagnostic_records(reason):
   if (record.get('status')==4 and record.get('reached_goal') is True
       and all(isinstance(record.get(key),(int,float)) and math.isfinite(record[key])
               for key in ('measured','result_position'))
       and abs(record['measured']-record['result_position'])>.003):return True
 return False
def status(row):
 s=row['stop_reason']
 if s=='all_blocks_stacked':return '전부 적층'
 if s=='time_limit_300s':return '5분 종료'
 if s=='no_feasible_direct_pick':return '경로 없음'
 if any(code in s for code in ('JOINT_FEEDBACK_STALE','stale joint feedback','Missing or stale measured joints')):return '관절 피드백 중단'
 if 'ARM_ENDPOINT_MISMATCH' in s:return '팔 도착값 불일치'
 if 'STANDSTILL_UNCONFIRMED_AFTER_MONITOR_PREWARM' in s:return '정지 피드백 확인 실패'
 if gripper_feedback_mismatch(s):return '그리퍼 결과·관절 피드백 불일치'
 return '기타 중단'
def brief_reason(row):
 records=list(diagnostic_records(row['stop_reason']))
 if status(row)=='그리퍼 결과·관절 피드백 불일치':
  record=next(r for r in records if 'measured' in r and 'result_position' in r)
  return f"명령 성공 응답 위치 {record['result_position']}rad와 실행기가 읽은 위치 {record['measured']}rad가 다름."
 if status(row)=='팔 도착값 불일치':
  return '팔 동작 성공 응답 후, 실행기가 확인한 관절 오차가 도착 허용값 0.001rad를 초과함.'
 record=next((r for r in records if 'code' in r),None)
 if record:return record['code']+(' · 단계 '+record['stage'] if record.get('stage') else '')
 return row['stop_reason'].split('{',1)[0].strip()[:240]
normal=[r for r in rows if status(r) in ('5분 종료','전부 적층')]
failed=[r for r in rows if r not in normal]
normal_label=f"5분 도달 {sum(status(r)=='5분 종료' for r in rows)}개, 전체 적층 완료 {sum(status(r)=='전부 적층' for r in rows)}개, 조기 중단 {len(failed)}개"
summary=dict(completed=32,normal=len(normal),stopped_or_failed=len(failed),
 floor_min=min(r['highest_layer'] for r in rows),floor_max=max(r['highest_layer'] for r in rows),
 floor_mean=round(statistics.mean(r['highest_layer'] for r in rows),2),
 added_blocks_mean=round(statistics.mean(r['highest_layer']-1 for r in rows),2),
 generated_at=datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
 stopped_by_reason={label:sum(status(r)==label for r in failed) for label in
                    ('관절 피드백 중단','그리퍼 결과·관절 피드백 불일치','팔 도착값 불일치','정지 피드백 확인 실패','경로 없음','기타 중단')},
 measurement_history=measurement_history,
 failed_cases=[dict(r,classification=status(r)) for r in failed])
(ROOT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
for name,selected in [('P0_32개_결과.csv',rows),('P0_중단항목.csv',failed)]:
 with (ROOT/name).open('w',encoding='utf-8-sig',newline='') as output:
  writer=csv.writer(output)
  writer.writerow(['번호','장애물','목표','최고층(바닥 포함)','추가 적층 수','후퇴 완료층','실행 시간(초)','분류','종료 사유','원본 로그 폴더'])
  for r in selected:
   writer.writerow([r['number'],r['obstacle'],r['target'],r['highest_layer'],r['highest_layer']-1,
                    r['complete_cycle_layer'],r['elapsed_seconds'],status(r),r['stop_reason'],
                    f"cases/{r['number']:02d}_obstacle_{r['obstacle']}_target_{r['target']}"])
text=['P0 32개 RViz 가상 시험 결과',summary['generated_at'],
 '시험 시간: 배치당 300초. 장면 생성·최초 계획·충돌 검사·실행·후속 계획을 포함.',
 '층수는 바닥 블록 1개를 포함. 제한 시각 전에 놓기와 장면 반영이 완료된 층만 인정.',
 'complete_cycle_layer는 후퇴까지 완료한 층수. 마지막 놓기와 다를 수 있음.',
 '환경: 별도 ROS domain 176~179, localhost, 복사한 v22 코드, 4개 병렬, 후순위 CPU 마지막 8개.',
 *([measurement_history] if measurement_history else []),
 '하드웨어: GenericSystem 위치/속도 모사, calculate_dynamics=true. 실물 연결 없음.',
 '가상 그리퍼 완료 응답: 백엔드 결과 후 최소 0.35초 경과하고, 완료 순간 최신 위치 피드백이 결과 위치와 일치해야 함.',
 '이 조건은 실행기 자신의 관절 구독이 같은 표본을 처리했다는 확인은 아님. 실제 AG95 이동/접촉/미끄럼은 미모사.',
 'RViz는 표시 도구이며 블록 동역학·탑 진동·낙하를 계산하지 않음. 실물 5분 결과와 구분.',
 normal_label+'.',
 f"전체 시험 최소 {summary['floor_min']}층 / 최대 {summary['floor_max']}층 / 평균 {summary['floor_mean']}층. 중단 시험의 관측 층수도 포함.",
 '피드백 중단의 원인이 CPU 부하·통신·실행기 처리 중 무엇인지는 이 기록만으로 확정하지 않음. 경로의 충돌 실패와 구분.',
 '번호,장애물,목표,5분 내 최고층,후퇴 완료층,결과']
text.extend(f"{r['number']},{r['obstacle']},{r['target']},{r['highest_layer']},{r['complete_cycle_layer']},{status(r)}" for r in rows)
text+=['','중단 상세']+[f"{r['number']}번 {r['obstacle']}→{r['target']}: {r['stop_reason']}" for r in failed]
(ROOT/'P0_32개_RViz_5분_시험결과.txt').write_text('\n'.join(text)+'\n')
style='''<style>body{font-family:"Noto Sans CJK KR";font-size:10pt;color:#15253a}h1{font-size:19pt;color:#163b63}h2{font-size:13pt;color:#163b63}p{line-height:135%}table{width:100%;border-collapse:collapse;font-size:9pt}th{background:#163b63;color:white}td,th{border:1px solid #cbd5e1;padding:5px}small{font-size:8pt;color:#475569}</style>'''
parts=[style,'<h1>P0 32개 · 5분 RViz 가상 시험</h1>',f'<p>실행 정책: {html.escape(manifest["policy"])}<br>작성: {summary["generated_at"]}</p>',
 f'<h2>전체 시험: 최소 {summary["floor_min"]}층 · 최대 {summary["floor_max"]}층 · 평균 {summary["floor_mean"]}층</h2>',
 f'<p>{normal_label}. 통계에는 중단 시험의 관측 층수도 포함한다. 바닥 1개 포함 층수이며, 새로 적층한 블록 수는 층수에서 1을 뺀 값이다.</p>',
 '<p>배치당 300초에 장면 생성, 경로 계획, 충돌 검사, 실행과 후속 계획을 포함했다. 제한 시각 전에 놓기와 장면 반영이 끝난 층을 기록했다. 후퇴 완료층은 마지막 놓기 층보다 낮을 수 있다.</p>',
 '<p>별도 ROS domain 176~179, localhost, 복사한 v22 코드로 4개 배치를 병렬 실행했다. GenericSystem의 위치·속도 피드백을 사용했고 실제 로봇과 연결하지 않았다. 가상 그리퍼는 백엔드 결과 후 최소 0.35초가 지나고, 완료 순간 최신 위치 피드백이 결과 위치와 일치해야 완료 응답을 보낸다. 실행기의 관절 구독이 같은 표본을 처리했는지는 별도로 확인하지 않는다.</p>',
 '<p><b>해석 범위:</b> RViz는 표시 도구다. 블록의 진동·낙하·접촉력·실제 AG95 파지각 변동은 모사하지 않는다. 표의 층수는 가상 제어기와 모델 검사 결과이며 실물 성적으로 확정할 수 없다. 병렬 실행의 자원 경쟁도 계획 시간에 포함된다.</p>']
if measurement_history:parts.append('<p><b>측정 경위:</b> '+html.escape(measurement_history)+'</p>')
parts.append('<p><small>개별 로그·RViz 화면: cases/번호_obstacle_장애물_target_목표/<br>가상 시험은 원본 PDF와 실물 작업 상태를 변경하지 않았다. 설정 지문은 manifest.json에 보관했다.</small></p>')
for start in [0,16]:
 parts.append('<h2 style="page-break-before:always"'+f'>P0 {start+1}~{start+16}</h2>')
 parts.append('<table><thead><tr><th>번호</th><th>장애물</th><th>목표</th><th>최고층</th><th>후퇴 완료</th><th>결과</th></tr></thead><tbody>')
 for r in rows[start:start+16]:
  values=[r['number'],r['obstacle'],r['target'],r['highest_layer'],r['complete_cycle_layer'],status(r)]
  parts.append('<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in values)+'</tr>')
 parts.append('</tbody></table>')
if failed:
 parts.append('<h2 style="page-break-before:always">중단 원인</h2>')
 parts.append('<p>관절 피드백 중단, 그리퍼 결과·관절 피드백 불일치, 팔 도착값 불일치와 정지 피드백 확인 실패는 기록된 실행 피드백 문제를 구분한 것이다. CPU 부하·통신·실행기 처리 중 어느 것이 원인인지는 이 기록만으로 확정하지 않는다. 경로 없음은 다음 직접 집기 경로를 찾지 못한 종료이며, 피드백 문제와 구분한다.</p>')
 for r in failed:parts.append(f'<p><b>{r["number"]}번 · 장애물 {r["obstacle"]} → 목표 {r["target"]} · {r["highest_layer"]}층 · {html.escape(status(r))}</b><br><small>{html.escape(brief_reason(r))}</small></p>')
 parts.append('<p>원문 진단은 같은 폴더의 P0_중단항목.csv와 P0_32개_RViz_5분_시험결과.txt에 보관했다.</p>')
content=''.join(parts);(ROOT/'P0_32개_RViz_5분_시험결과.html').write_text(content)
app=QApplication([]);document=QTextDocument();document.setDefaultFont(QFont('Noto Sans CJK KR',10));document.setHtml(content)
printer=QPrinter(QPrinter.HighResolution);printer.setOutputFormat(QPrinter.PdfFormat);printer.setPageSize(QPrinter.A4);printer.setPageMargins(14,14,14,14,QPrinter.Millimeter);printer.setOutputFileName(str(ROOT/'P0_32개_RViz_5분_시험결과.pdf'));document.print_(printer)
print(json.dumps({k:v for k,v in summary.items() if k!='failed_cases'},ensure_ascii=False))
